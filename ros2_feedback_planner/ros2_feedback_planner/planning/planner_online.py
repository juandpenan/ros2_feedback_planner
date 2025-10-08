"""This module defines the LlamaPlanner class for online planning using an LLM."""

from .planner_base import PlannerBase
import asyncio
import openai


class OnlinePlanner(PlannerBase):
    """Planner that uses OpenAI's ChatGPT for online planning."""

    def __init__(self, api_key=None, base_url=None):
        """Initialize the OnlinePlanner with the provided OpenAI API key and base URL."""

        self.client = openai.AsyncOpenAI(api_key=api_key, base_url=base_url)

    async def plan_async(self):
        """Async version of plan method."""

        response = await self.client.chat.completions.create(
            model='gemini-2.5-flash',
            messages=[
                {"role": "developer", "content": "You are a helpful assistant."},
                {"role": "user", "content": 'hey how are you?'}
            ]
        )
        return response.choices[0].message.content.strip().split('\n')
   
    
    def plan(self, goal: str = None, world_state: dict = None):
        # prompt = 'Hey how are you ?'

        # response = self.client.chat.completions.create(
        #     model='gemini-2.5-flash',
        #     messages=[
        #         {"role": "developer", "content": "You are a helpful assistant."},
        #         {"role": "user", "content": "Hello!"}
        #     ]
        # )
        return asyncio.run(self.plan_async())

