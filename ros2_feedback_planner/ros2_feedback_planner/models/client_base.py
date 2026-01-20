
"""This module provides an abstract base client for interfacing with OpenAI, Gemini, Hugging Face, or a local Llama model."""

import os
from openai import OpenAI
from abc import ABC
from google import genai
import base64
from io import BytesIO
import PIL.Image
from google.genai import types
from pydantic import BaseModel
import json
import asyncio
from datetime import datetime
from contextlib import asynccontextmanager
from huggingface_hub import InferenceClient



class BaseClient(ABC):

    """Abstract base client for interfacing with OpenAI, Gemini, or a local Llama model."""

    def __init__(
        self,
        vendor: str,
        api_key_variable: str = None,
        model_name: str = None,
        temperature: float = 0.0,
        top_p: float = 1.0,
        top_k: int = 40,
        max_tokens: int = 5000
    ):
        """
        Initialize the BaseClient with the specified vendor and generation configs.

        Args:
            vendor (str): The vendor to use ('openai', 'gemini', 'huggingface', or 'local').
            api_key_variable (str, optional): The API env variable where the key is stored.
            model_name (str, optional): The name of the local model to use.
            temperature (float, optional): Sampling temperature.
            top_p (float, optional): Nucleus sampling probability.
            top_k (int, optional): Top-k sampling.
            max_tokens (int, optional): Maximum number of tokens to generate.
        """
        if not model_name:
            raise ValueError('model_name must be provided')
        if not vendor:
            raise ValueError('vendor must be provided')
        
        self._input_queue = asyncio.LifoQueue(maxsize=15)
        self.vendor = vendor.lower()
        self._last_response = None
        self._system_prompt = None
        self._output_format_class = None
        self.model_name = model_name

        # generation configs
        self.temperature = temperature
        self.top_p = top_p
        self.top_k = top_k
        self.max_tokens = max_tokens

        if self.vendor == 'huggingface':
            # Use Hugging Face Inference API
            self.client = InferenceClient()
            self.history = []
            print(f'✓ Hugging Face client ready for model: {model_name}')
        elif self.vendor == 'local':
            # Use fast PyTorch inference with transformers instead of llama.cpp
            try:
                import torch
                from transformers import AutoProcessor, Idefics3ForConditionalGeneration
            except ImportError:
                raise ImportError(
                    'Local inference requires torch and transformers. '
                    'You can install them with `pip install torch transformers`.'
                )
            
            # Load processor
            self.processor = AutoProcessor.from_pretrained(model_name)
            
            # Load model with FP16 optimization
            print(f"Loading local model: {model_name} with FP16...")
            self.client = Idefics3ForConditionalGeneration.from_pretrained(
                model_name,
                torch_dtype=torch.float16,  # FP16 for speed
                device_map="auto",
                low_cpu_mem_usage=True
            )
            self.client.eval()
            
            # Compile model for faster inference (PyTorch 2.0+)
            if hasattr(torch, 'compile'):
                print("Compiling model with torch.compile()...")
                try:
                    self.client = torch.compile(self.client, mode="reduce-overhead")
                    print("✓ Model compiled successfully")
                except Exception as e:
                    print(f"⚠ Compilation failed: {e}")
                    print("  Continuing without compilation")
            
            self.history = []
            print("✓ Local model ready for inference")
        elif self.vendor == 'openai':
            self.client = OpenAI(api_key=os.environ[api_key_variable])
        elif self.vendor == 'gemini':
            config = types.GenerateContentConfig(
                temperature=self.temperature,
                top_p=self.top_p,
                top_k=self.top_k,
                max_output_tokens=self.max_tokens,
            )
            # used only for 'real time gemini features'
            self.live_client = genai.Client(
                api_key=os.environ[api_key_variable])
            self._gemini_base_client = genai.Client(
                api_key=os.environ[api_key_variable])
            self.client = self._gemini_base_client.chats.create(model=self.model_name,
                                                                config=config)
        else:
            raise ValueError(f'Unknown vendor: {vendor}')

    def generate(self, prompt: str, image: PIL.Image = None) -> str:
        if self.vendor == 'huggingface':
            # Format messages for Hugging Face
            messages = []

            # Add system prompt if set
            if self._system_prompt:
                messages.append({
                    'role': 'system',
                    'content': self._system_prompt
                })

            # Prepare user message content
            if image is not None:
                # Convert image to base64
                buffered = BytesIO()
                image.save(buffered, format='JPEG')
                image_data = base64.b64encode(buffered.getvalue()).decode('utf-8')
                
                # Multi-modal message with text and image
                messages.append({
                    'role': 'user',
                    'content': [
                        {'type': 'text', 'text': prompt},
                        {
                            'type': 'image_url',
                            'image_url': {
                                'url': f'data:image/jpeg;base64,{image_data}'
                            }
                        }
                    ]
                })
            else:
                # Text-only message
                messages.append({
                    'role': 'user',
                    'content': prompt
                })

            # Call Hugging Face Inference API (non-streaming)
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
            )

            response_text = response.choices[0].message.content
            self._last_response = response_text
            
            # Update history
            self.history.append({
                'role': 'user',
                'content': prompt
            })
            self.history.append({
                'role': 'assistant',
                'content': response_text
            })
            
            return response_text

        elif self.vendor == 'local':
            import torch

            # Format messages for chat template
            messages = []

            # Add system prompt if set
            if self._system_prompt:
                messages.append({
                    'role': 'system',
                    'content': self._system_prompt
                })

            # Add user message with image if provided
            content = []
            if image is not None:
                content.append({'type': 'image'})
            content.append({'type': 'text', 'text': prompt})

            messages.append({
                'role': 'user',
                'content': content
            })

            # Apply chat template
            prompt_text = self.processor.apply_chat_template(
                messages, 
                add_generation_prompt=True
            )

            # Prepare image list
            images = [image] if image is not None else None

            # Process inputs
            inputs = self.processor(
                text=prompt_text, 
                images=images, 
                return_tensors="pt"
            )
            inputs = {k: v.to(self.client.device) for k, v in inputs.items()}

            # Generate with optimizations
            with torch.inference_mode():  # Faster than no_grad
                generated_ids = self.client.generate(
                    **inputs,
                    max_new_tokens=self.max_tokens,
                    do_sample=self.temperature > 0,
                    temperature=self.temperature if self.temperature > 0 else None,
                    top_p=self.top_p if self.temperature > 0 else None,
                    use_cache=True  # Enable KV cache
                )

            generated_text = self.processor.batch_decode(
                generated_ids,
                skip_special_tokens=True
            )[0]
            
            if "Assistant:" in generated_text:
                response = generated_text.split("Assistant:")[-1].strip()
            else:
                response = generated_text.strip()
            
            self._last_response = response
            
            self.history.append({
                'role': 'user',
                'content': prompt
            })
            self.history.append({
                'role': 'assistant',
                'content': response
            })
            
            return response

        elif self.vendor == 'openai':

            messages = [{'role': 'user', 'content': prompt}]

            if image is not None:
                buffered = BytesIO()
                image.save(buffered, format='PNG')
                img_str = base64.b64encode(buffered.getvalue()).decode()
                messages[-1]['content'] = [
                    {'type': 'text', 'text': prompt},
                    {'type': 'input_image', 'image_url': f'data:image/png;base64,{img_str}'}
                ]

            if self._output_format_class:
                response = self.client.responses.parse(
                    model=self.model_name,
                    temperature=self.temperature,
                    top_p=self.top_p,
                    top_k=self.top_k,
                    instructions=self._system_prompt if self._system_prompt else None,
                    max_output_tokens=self.max_tokens,
                    previous_response_id=self._last_response.id,
                    messages=messages,
                    text_format=self._output_format_class
                )
                self._last_response = response
                return response.output_parsed
            else:
                response = self.client.responses.create(
                    model=self.model_name,
                    temperature=self.temperature,
                    top_p=self.top_p,
                    top_k=self.top_k,
                    instructions=self._system_prompt if self._system_prompt else None,
                    max_output_tokens=self.max_tokens,
                    previous_response_id=self._last_response.id,
                    messages=messages
                )
                self._last_response = response
                return response.output_text

        elif self.vendor == 'gemini':
            message = [prompt]

            if image is not None:
                message.append(image)

            try:
                response = self.client.send_message(
                    message,
                    config=types.GenerateContentConfig(
                        response_mime_type='application/json',
                        system_instruction=self._system_prompt if self._system_prompt else None,
                        response_schema=self._output_format_class if self._output_format_class else None,
                        thinking_config=types.ThinkingConfig(thinking_budget=-1)
                    ),
                )
            except Exception as e:
                print(f'Error during Gemini send_message: {e}')
                response = None
                return response
            return response.text

    def set_system_prompt(self, prompt: str):
        """
        Set a system prompt for the conversation history.

        Args:
            prompt (str): The system prompt to set.
        """
        if self.vendor == 'huggingface':
            self._system_prompt = prompt
        elif self.vendor == 'local':
            self.history.insert(0, {'role': 'system', 'content': prompt})
        elif self.vendor == 'openai':
            self._system_prompt = prompt
        elif self.vendor == 'gemini':
            self._system_prompt = prompt
        else:
            raise ValueError(f'Unknown vendor: {self.vendor}')

    def set_output_format(self, format_class: BaseModel):
        """
        Set the desired output format for model responses.

        Args:
            format_class (type[pydantic.BaseModel]): The Pydantic model class to use for output formatting.
        """

        if self.vendor == 'local':
            self._output_format_class = json.dumps(format_class, indent=2)
            return
        self._output_format_class = format_class
    
    async def start_live_session(self, config):
            """
            Gemini real-time streaming generation.
            Only works if vendor == 'gemini'.
            """
            if self.vendor != 'gemini':
                raise NotImplementedError('Real-time streaming only supported for Gemini.')

            # config = types.GenerateContentConfig(
            #     temperature=self.temperature,
            #     top_p=self.top_p,
            #     top_k=self.top_k,
            #     max_output_tokens=self.max_tokens,
            # )
            async with self.live_client.aio.live.connect(
                model=self.model_name,
                config=config
            ) as session:
                while True:
                    prompt, image = await self._input_queue.get()
                    if prompt is not None and image is not None:
                        await session.send_realtime_input(media=image)  
                        await session.send_realtime_input(text=prompt)
                    elif prompt is not None and image is None:
                        await session.send_realtime_input(text=prompt)
                    elif image is not None and prompt is None:
                        await session.send_realtime_input(media=image)
                    else:
                        await asyncio.sleep(0.25)
                        continue
                    response_buffer = ''
                    async for response in session.receive():
                        if response.text is None:
                            await asyncio.sleep(0.25)
                            continue
                        print(response.text)
                        response_buffer += response.text
                        self._last_response = response_buffer
                    await asyncio.sleep(0.25)

    def send_rt_input(self, prompt, image=None):
        try:
            self._input_queue.put_nowait((prompt, image))
        except asyncio.QueueFull:
            return

    def clear_rt_buffer(self):
        self._last_response = ''
        while not self._input_queue.empty():
            self._input_queue.get_nowait()
            self._input_queue.task_done()

    def get_latest_response(self):
        return self._last_response

    @asynccontextmanager
    async def live_session(self, config):
        if self.vendor != 'gemini':
            # todo, change the local with a basic client of openai since its way faster using llamacpp + client
            raise NotImplementedError('Real-time streaming only supported for Gemini.')
        config['temperature'] = self.temperature
        config['top_p'] = self.top_p
        config['top_k'] = self.top_k
        config['max_output_tokens'] = self.max_tokens
        async with self.live_client.aio.live.connect(
            model=self.model_name,
            config=config
        ) as session:
            yield session
