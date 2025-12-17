"""
Helper script to delete and recreate Qdrant collection with correct dimensions
Run this before ingesting with new embeddings
"""

from qdrant_client import QdrantClient
from config import get_settings

settings = get_settings()

# Connect to Qdrant
if settings.qdrant_url and "http" in settings.qdrant_url:
    print(f"Connecting to Qdrant Cloud: {settings.qdrant_url}")
    qdrant = QdrantClient(
        url=settings.qdrant_url, 
        api_key=settings.qdrant_api_key
    )
else:
    print(f"Using Local Qdrant Storage: {settings.qdrant_path}")
    qdrant = QdrantClient(path=settings.qdrant_path)

collection_name = settings.qdrant_collection

# Check if collection exists
collections = qdrant.get_collections()
exists = any(c.name == collection_name for c in collections.collections)

if exists:
    print(f"Deleting existing collection: {collection_name}")
    qdrant.delete_collection(collection_name=collection_name)
    print(f"✅ Collection '{collection_name}' deleted successfully!")
    print("\nYou can now run: python -m rag.ingestion")
else:
    print(f"Collection '{collection_name}' does not exist. Nothing to delete.")
    print("You can proceed with: python -m rag.ingestion")
