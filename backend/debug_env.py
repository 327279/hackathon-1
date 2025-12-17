from config import get_settings
import os

print(f"Current Working Directory: {os.getcwd()}")
print(f"Checking for .env file: {os.path.exists('.env')}")

settings = get_settings()

print("\n--- Configuration Debug ---")
print(f"Database URL: {settings.database_url[:10]}... (redacted)")
print(f"Qdrant URL: {settings.qdrant_url}")
print(f"OpenAI Key starts with: {settings.openai_api_key[:3]}..." if settings.openai_api_key else "OpenAI Key: MISSING")
