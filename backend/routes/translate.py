"""
Translation Routes
"""

from fastapi import APIRouter
from pydantic import BaseModel
from config import get_settings
import google.generativeai as genai

router = APIRouter(prefix="/api/translate", tags=["translate"])
settings = get_settings()
genai.configure(api_key=settings.gemini_api_key)

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
    
    model = genai.GenerativeModel(settings.gemini_model)
    prompt = f"{system_prompt}\n\nContent to translate:\n{request.content}"
    response = model.generate_content(prompt)
    
    return TranslateResponse(translated_content=response.text)
