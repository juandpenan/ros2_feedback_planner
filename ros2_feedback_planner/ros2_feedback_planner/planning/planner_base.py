

from google import genai
import time
from google.genai import types

with open('/home/juan/Downloads/warehouse_sample_2.jpeg', 'rb') as f:
    image_bytes = f.read()

client = genai.Client()
start_time = time.time()
for chunk in client.models.generate_content_stream(
        model="gemini-2.0-flash-lite",
        contents=[
            types.Part.from_bytes(
                data=image_bytes,
                mime_type='image/jpeg',
            ),
            'Caption this image'
            ]
    ):
    print(chunk.text)
    elapsed_time = time.time() - start_time
    break
print(f"Elapsed time: {elapsed_time:.4f} seconds")

