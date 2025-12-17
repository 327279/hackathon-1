"""
Translation Routes
"""

from fastapi import APIRouter
from pydantic import BaseModel
from config import get_settings
from openai import OpenAI

router = APIRouter(prefix="/api/translate", tags=["translate"])
settings = get_settings()
client = OpenAI(api_key=settings.openai_api_key)

class TranslateRequest(BaseModel):
    content: str
    target_language: str = "Urdu"

class TranslateResponse(BaseModel):
    translated_content: str

@router.post("/", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    
    system_prompt = f"""You are a professional technical translator.
    Translate the following technical documentation into {request.target_language}.
    - IMPORTANT: Keep all code blocks intact and untranslated.
    - IMPORTANT: Keep all technical terms (ROS Node, Topic, Service) in English but transliterated if appropriate, or keep English script if standard.
    - Maintain Markdown formatting exactly.
    """
    
    completion = client.chat.completions.create(
        model=settings.openai_model,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": request.content}
        ]
    )
    
    return TranslateResponse(translated_content=completion.choices[0].message.content)
