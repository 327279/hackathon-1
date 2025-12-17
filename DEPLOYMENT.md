# Deployment Guide - Physical AI & Humanoid Robotics Textbook

This guide covers deploying both the frontend (Docusaurus) and backend (FastAPI) to production.

## 🎯 Quick Deployment Options

| Component | Recommended Platform | Alternative |
|-----------|---------------------|-------------|
| Frontend | Vercel | GitHub Pages, Netlify |
| Backend | Render | Railway, Fly.io, AWS |
| Database | Neon (Postgres) | Supabase, Railway Postgres |
| Vector DB | Qdrant Cloud | Self-hosted Qdrant |

## 📦 Frontend Deployment

### Option 1: Vercel (Recommended)

1. **Push to GitHub:**
   ```bash
   git add .
   git commit -m "Ready for deployment"
   git push origin main
   ```

2. **Deploy on Vercel:**
   - Go to [vercel.com](https://vercel.com)
   - Click "New Project"
   - Import your GitHub repository
   - Configure:
     - **Framework Preset:** Other
     - **Build Command:** `npm run build`
     - **Output Directory:** `build`
     - **Install Command:** `npm install`
   - Click "Deploy"

3. **Update Backend CORS:**
   After deployment, add your Vercel URL to `backend/main.py`:
   ```python
   allow_origins=[
       "http://localhost:3000",
       "https://your-project.vercel.app",  # Add this
   ],
   ```

### Option 2: GitHub Pages

1. **Configure `docusaurus.config.ts`:**
   ```typescript
   const config: Config = {
     url: 'https://yourusername.github.io',
     baseUrl: '/repository-name/',
     organizationName: 'yourusername',
     projectName: 'repository-name',
     deploymentBranch: 'gh-pages',
     trailingSlash: false,
   };
   ```

2. **Deploy:**
   ```bash
   GIT_USER=yourusername npm run deploy
   ```

3. **Enable GitHub Pages:**
   - Go to repository Settings > Pages
   - Source: Deploy from branch `gh-pages`

### Option 3: Netlify

1. **Create `netlify.toml`:**
   ```toml
   [build]
     command = "npm run build"
     publish = "build"
   
   [[redirects]]
     from = "/*"
     to = "/index.html"
     status = 200
   ```

2. **Deploy:**
   - Connect repository on [netlify.com](https://netlify.com)
   - Netlify auto-detects Docusaurus
   - Click "Deploy"

## 🖥️ Backend Deployment

### Option 1: Render (Recommended)

1. **Create `render.yaml`:**
   ```yaml
   services:
     - type: web
       name: physical-ai-backend
       runtime: python
       buildCommand: "pip install -r requirements.txt"
       startCommand: "uvicorn main:app --host 0.0.0.0 --port $PORT"
       envVars:
         - key: PYTHON_VERSION
           value: 3.10.0
         - key: OPENAI_API_KEY
           sync: false
         - key: DATABASE_URL
           fromDatabase:
             name: physical-ai-db
             property: connectionString
         - key: QDRANT_URL
           sync: false
         - key: QDRANT_API_KEY
           sync: false
         - key: JWT_SECRET
           generateValue: true
   
   databases:
     - name: physical-ai-db
       databaseName: physical_ai
       user: physical_ai_user
   ```

2. **Deploy:**
   - Push to GitHub
   - Go to [render.com](https://render.com)
   - Click "New Blue Blueprint"
   - Connect repository
   - Fill in environment variables
   - Deploy

3. **Add Environment Variables:**
   - `OPENAI_API_KEY` - Your OpenAI key
   - `QDRANT_URL` - Your Qdrant cloud URL
   - `QDRANT_API_KEY` - Your Qdrant API key
   - `JWT_SECRET` - Auto-generated or custom

4. **Run Ingestion:**
   After first deployment, run ingestion manually via Render shell or locally pointing to production Qdrant.

### Option 2: Railway

1. **Install Railway CLI:**
   ```bash
   npm install -g @railway/cli
   ```

2. **Initialize:**
   ```bash
   cd backend
   railway login
   railway init
   ```

3. **Add Services:**
   - Add PostgreSQL database
   - Deploy backend service

4. **Set Variables:**
   ```bash
   railway variables set OPENAI_API_KEY=your_key
   railway variables set QDRANT_URL=your_url
   railway variables set QDRANT_API_KEY=your_key
   ```

5. **Deploy:**
   ```bash
   railway up
   ```

### Option 3: Docker + Any Platform

1. **Create `Dockerfile`:**
   ```dockerfile
   FROM python:3.10-slim
   
   WORKDIR /app
   
   COPY requirements.txt .
   RUN pip install --no-cache-dir -r requirements.txt
   
   COPY . .
   
   CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
   ```

2. **Create `.dockerignore`:**
   ```
   __pycache__
   *.pyc
   venv
   .env
   ```

3. **Build and Push:**
   ```bash
   docker build -t your-username/physical-ai-backend .
   docker push your-username/physical-ai-backend
   ```

4. **Deploy to:**
   - Fly.io: `fly launch`
   - AWS ECS, Google Cloud Run, Azure Container Apps, etc.

## 🗄️ Database Setup

### Neon (Recommended)

1. **Create Database:**
   - Go to [neon.tech](https://neon.tech)
   - Create new project
   - Copy connection string

2. **Set Environment Variable:**
   ```env
   DATABASE_URL=postgresql://user:password@host/database?sslmode=require
   ```

3. **Run Migrations:**
   - Tables auto-create on first request
   - Or use Alembic for production:
     ```bash
     pip install alembic
     alembic init alembic
     alembic revision --autogenerate -m "Initial"
     alembic upgrade head
     ```

### Supabase Alternative

1. Create project at [supabase.com](https://supabase.com)
2. Get connection string from Settings > Database
3. Use as `DATABASE_URL`

## 🔍 Vector Database (Qdrant)

### Qdrant Cloud (Recommended)

1. **Create Cluster:**
   - Go to [cloud.qdrant.io](https://cloud.qdrant.io)
   - Create cluster (Free tier available)
   - Note URL and API key

2. **Set Variables:**
   ```env
   QDRANT_URL=https://xxxxx.qdrant.io
   QDRANT_API_KEY=your_api_key
   ```

3. **Run Ingestion:**
   ```bash
   cd backend
   python -m rag.ingestion
   ```

### Self-Hosted Qdrant

1. **Using Docker:**
   ```bash
   docker run -p 6333:6333 \
     -v $(pwd)/qdrant_storage:/qdrant/storage \
     qdrant/qdrant
   ```

2. **Configure:**
   ```env
   QDRANT_URL=http://localhost:6333
   # No API key needed for local
   ```

## 🔐 Environment Variables

### Production Checklist

**Frontend (if using API calls):**
- `REACT_APP_API_URL` - Backend API URL

**Backend:**
- ✅ `OPENAI_API_KEY` - OpenAI API key
- ✅ `DATABASE_URL` - PostgreSQL connection string
- ✅ `QDRANT_URL` - Qdrant instance URL
- ✅ `QDRANT_API_KEY` - Qdrant authentication
- ✅ `JWT_SECRET` - Strong random string (use `openssl rand -hex 32`)
- ⚙️ `DEBUG` - Set to `false` in production

### Generating Secrets

```bash
# JWT Secret
openssl rand -hex 32

# Or in Python
python -c "import secrets; print(secrets.token_hex(32))"
```

## 🚀 Post-Deployment Steps

### 1. Update CORS Origins

In `backend/main.py`, add your frontend URL:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://your-frontend-domain.com",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### 2. Update API Endpoints in Frontend

If backend is on different domain, update all `fetch` calls:

```typescript
// src/components/Auth/index.tsx, ChatBot/index.tsx, etc.
const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

fetch(`${API_URL}/api/auth/signup`, {
  // ...
});
```

Or use environment variables in build:

```bash
# .env.production
REACT_APP_API_URL=https://your-backend.onrender.com
```

### 3. Run Content Ingestion

After deploying backend, ingest documentation:

```bash
# Locally, pointing to production Qdrant
cd backend
# Update .env with production QDRANT_URL and QDRANT_API_KEY
python -m rag.ingestion
```

### 4. Test Authentication Flow

1. Visit your deployed frontend
2. Click "Sign Up"
3. Create account
4. Complete onboarding
5. Test chatbot, personalization, translation

### 5. Monitor Logs

- **Vercel:** Check deployment logs in dashboard
- **Render:** View logs in service page
- **Backend:** Check for errors in API logs

## 📊 Performance Optimization

### Frontend

1. **Enable Compression:**
   Vercel and Netlify do this automatically.

2. **CDN Caching:**
   Static assets cached automatically on most platforms.

3. **Image Optimization:**
   - Use WebP format
   - Implement lazy loading

### Backend

1. **Connection Pooling:**
   SQLAlchemy already uses pooling. Adjust in `database.py`:
   ```python
   engine = create_async_engine(
       settings.database_url,
       pool_size=20,
       max_overflow=10
   )
   ```

2. **Caching:**
   Add Redis for frequently accessed data:
   ```bash
   pip install redis
   ```

3. **Rate Limiting:**
   ```bash
   pip install slowapi
   ```

## 🐛 Troubleshooting Deployment

### Frontend Build Fails

**Issue:** TypeScript errors
- **Fix:** Run `npm run typecheck` locally and fix errors

**Issue:** Missing modules
- **Fix:** Ensure all CSS modules exist for components

### Backend Won't Start

**Issue:** `ModuleNotFoundError`
- **Fix:** Verify all dependencies in `requirements.txt`

**Issue:** Database connection
- **Fix:** Check `DATABASE_URL` format and SSL mode

**Issue:** Qdrant connection
- **Fix:** Verify QDRANT_URL is accessible from server

### CORS Errors

- Add frontend domain to `allow_origins` in `main.py`
- Ensure `allow_credentials=True` is set
- Check backend is accessible from frontend domain

### RAG Not Working

1. Verify Qdrant collection exists
2. Re-run ingestion
3. Check OpenAI API key is valid
4. Test query endpoint directly

## 🔄 Continuous Deployment

### GitHub Actions (Frontend)

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to Vercel

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: amondnet/vercel-action@v20
        with:
          vercel-token: ${{ secrets.VERCEL_TOKEN }}
          vercel-org-id: ${{ secrets.ORG_ID }}
          vercel-project-id: ${{ secrets.PROJECT_ID }}
```

### Auto-Deploy (Backend)

Most platforms (Render, Railway) auto-deploy on git push to main.

## 📞 Support

If deployment issues persist:
1. Check platform-specific documentation
2. Review application logs
3. Verify all environment variables
4. Test locally with production settings

---

**Deployment complete! 🎉 Your Physical AI textbook is now live!**
