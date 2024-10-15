"""from starlette.applications import Starlette
from starlette.responses import JSONResponse
from starlette.routing import Route
from transformers import pipeline
import asyncio

async def homepage(request):
    payload = await request.body()
    string = payload.decode("utf-8")
    response_q = asyncio.Queue()
    await request.app.model_queue.put((string, response_q))
    output = await response_q.get()
    return JSONResponse(output)

async def server_loop(q):
    pipe = pipeline(model="google-bert/bert-base-uncased")
    while True:
        (string, response_q) = await q.get()
        out = pipe(string)
        await response_q.put(out)

app = Starlette(
    routes=[
        Route("/", homepage, methods=["POST"]),
    ],
)

async def startup_event():
    q = asyncio.Queue()
    app.model_queue = q
    asyncio.create_task(server_loop(q))

app.add_event_handler("startup", startup_event)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)"""
import os
import torch
from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM
from starlette.applications import Starlette
from starlette.responses import JSONResponse
from starlette.routing import Route
import asyncio

# Check if CUDA is available and set the device
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# Load the model and tokenizer
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(CURRENT_DIR, "controller/Meta-Llama-3-8B-Instruct/")
tokenizer = AutoTokenizer.from_pretrained(model_path)
model = AutoModelForCausalLM.from_pretrained(model_path, torch_dtype=torch.float16)

# Create the pipeline
generator = pipeline(
    "text-generation",
    model=model,
    tokenizer=tokenizer,
    device=device
)

async def generate_text(prompt):
    generated_text = generator(prompt, max_length=5000, num_return_sequences=1)
    return generated_text[0]['generated_text']

async def homepage(request):
    payload = await request.body()
    prompt = payload.decode("utf-8")
    response_q = asyncio.Queue()
    await request.app.model_queue.put((prompt, response_q))
    output = await response_q.get()
    return JSONResponse({"response": output})

async def server_loop(q):
    while True:
        (prompt, response_q) = await q.get()
        output = await generate_text(prompt)
        await response_q.put(output)

app = Starlette(
    routes=[
        Route("/", homepage, methods=["POST"]),
    ],
)

async def startup_event():
    q = asyncio.Queue()
    app.model_queue = q
    asyncio.create_task(server_loop(q))

app.add_event_handler("startup", startup_event)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
