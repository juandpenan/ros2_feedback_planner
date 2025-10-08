
"""This module provides an abstract base client for interfacing with OpenAI, Gemini, or a local Llama model."""

import os
import fnmatch
from openai import OpenAI
from abc import ABC
from llama_cpp import Llama
from google import genai
import base64
from io import BytesIO
import PIL.Image
from google.genai import types
from pydantic import BaseModel
import json
import asyncio
from datetime import datetime
from pathlib import Path
from contextlib import asynccontextmanager



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
            vendor (str): The vendor to use ('openai', 'gemini', or 'local').
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

        if self.vendor == 'local':
            try:
                from huggingface_hub import HfFileSystem
                from huggingface_hub.utils import validate_repo_id
            except ImportError:
                raise ImportError(
                    'Llama.from_pretrained requires the huggingface-hub package. '
                    'You can install it with `pip install huggingface-hub`.'
                )
            validate_repo_id(model_name)

            hffs = HfFileSystem()

            files = [
                file['name'] if isinstance(file, dict) else file
                for file in hffs.ls(model_name, recursive=True)
            ]

            file_list = []

            for file in files:
                rel_path = Path(file).relative_to(model_name)
                file_list.append(str(rel_path))

            # find the only/first shard file:
            matching_files = [file for file in file_list if fnmatch.fnmatch(file, '*.gguf')]

            if len(matching_files) == 0:
                raise ValueError(
                    f'No file found in {model_name} that match *.gguf\n\n'
                    f'Available Files:\n{json.dumps(file_list)}'
                )

            self.client = Llama.from_pretrained(
                repo_id=model_name,
                filename=matching_files[0],
                n_gpu_layers=24,      # Partial GPU offload (enough to speed up, fits 8GB)
                n_batch=2048,         # Large batch but safe for 8GB
                n_ctx=2048,           # Typical max context size

                n_threads=12,         # Use CPU cores for other work
                n_threads_batch=12,

                use_mlock=True,
                use_mmap=True,
                verbose=False
            )
            self.history = []
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
            self.client = genai.Client(
                api_key=os.environ[api_key_variable]).chats.create(model=self.model_name,
                                                                   config=config)
        else:
            raise ValueError(f'Unknown vendor: {vendor}')

    def generate(self, prompt: str, image: PIL.Image = None) -> str:
        if self.vendor == 'local':

            self.history.append(
                {
                    'role': 'user',
                    'content': prompt
                }
            )

            if image is not None:
                buffered = BytesIO()
                image.save(buffered, format='PNG')
                img_str = base64.b64encode(buffered.getvalue()).decode()
                self.history[-1]['content'] = [
                    {'type': 'text', 'text': prompt},
                    {'type': 'image_url', 'image_url': f'data:image/png;base64,{img_str}'}
                ]

            response = self.client.create_chat_completion(
                messages=self.history,
                temperature=self.temperature,
                top_p=self.top_p,
                top_k=self.top_k,
                response_format={
                    'type': 'json_object',
                    'schema': self._output_format_class
                } if self._output_format_class else None,
            )
            # self.history += [{'role': el.role, 'content': el.content} for el in response.output]

            self._last_response = response

            return self._last_response

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
                        response_schema=self._output_format_class if self._output_format_class else None
                    ),
                )
            except Exception as e:
                print(f'Error during Gemini send_message: {e}')
                response = None
            return response.text

    def set_system_prompt(self, prompt: str):
        """
        Set a system prompt for the conversation history.

        Args:
            prompt (str): The system prompt to set.
        """
        if self.vendor == 'local':
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
