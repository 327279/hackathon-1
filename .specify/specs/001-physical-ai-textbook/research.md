# Research Document

## Metadata
- **Feature ID:** 001-physical-ai-textbook
- **Created:** 2025-12-16

---

## 1. Docusaurus v3 Research

### Key Features for This Project
- **MDX Support:** Allows React components in markdown (for ChatBot integration)
- **Sidebar Autogeneration:** From folder structure
- **Theme Customization:** Via swizzling
- **GitHub Pages Deployment:** Built-in support

### Initialization Command
```bash
npx create-docusaurus@latest physical-ai-textbook classic --typescript
```

### Swizzling for Custom Doc Pages
```bash
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
```

### Relevant Dependencies
- `@docusaurus/core`: ^3.6.0
- `@docusaurus/preset-classic`: ^3.6.0
- `prism-react-renderer`: For code highlighting

---

## 2. OpenAI Agents SDK Research

### Installation
```bash
pip install openai-agents
```

### Key Components
- **Agent:** Main class for creating AI agents
- **Tool:** For function calling
- **Runner:** For executing agent workflows

### RAG Pattern
```python
from openai_agents import Agent, Tool

agent = Agent(
    model="gpt-4o-mini",
    instructions="You are a helpful assistant for Physical AI learning.",
    tools=[search_knowledge_base]
)
```

---

## 3. Qdrant Cloud Research

### Free Tier Limits
- 1GB storage
- 100k vectors
- Sufficient for book content

### Python Client
```bash
pip install qdrant-client
```

### Collection Setup
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client = QdrantClient(url="...", api_key="...")
client.create_collection(
    collection_name="book_content",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
)
```

---

## 4. Neon Postgres Research

### Connection String Format
```
postgresql://user:password@host.neon.tech/dbname?sslmode=require
```

### SQLAlchemy Async
```python
from sqlalchemy.ext.asyncio import create_async_engine

engine = create_async_engine(
    "postgresql+asyncpg://...",
    echo=True
)
```

---

## 5. Better-Auth Research

### Installation (Frontend)
```bash
npm install better-auth
```

### Key Features
- Email/password authentication
- Session management
- React hooks for auth state

### Integration Pattern
```typescript
import { createAuth } from 'better-auth/react';

const auth = createAuth({
  baseURL: '/api/auth',
});
```

---

## 6. ROS 2 Content Research

### Key Topics for Module 1
1. **Nodes:** Independent executables
2. **Topics:** Publish/subscribe messaging
3. **Services:** Request/response pattern
4. **Actions:** Long-running tasks with feedback
5. **rclpy:** Python client library
6. **URDF:** Robot description format

### Code Example References
```python
# Basic ROS 2 node example
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

---

## 7. NVIDIA Isaac Research

### Key Components
- **Isaac Sim:** Photorealistic simulation (requires RTX GPU)
- **Isaac ROS:** Hardware-accelerated perception
- **Nav2:** Navigation stack for ROS 2

### Hardware Requirements
- NVIDIA RTX 4070 Ti+ (12GB VRAM minimum)
- 64GB RAM recommended
- Ubuntu 22.04 LTS

---

*Research compiled for Spec-Kit Plus planning phase*
