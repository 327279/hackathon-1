# Physical AI & Humanoid Robotics Textbook 🤖

<a comprehensive, interactive textbook platform for learning Physical AI, Humanoid Robotics, ROS 2, and related technologies. Built with Docusaurus, enhanced with AI-powered features including RAG chatbot, personalized content, and multi-language support.

## ✨ Features

### 📚 Comprehensive Content
- **Module 1: ROS 2** - Robot Operating System fundamentals, nodes, topics, services, and URDF
- **Module 2: Simulation** - Gazebo, Unity, and physics-based simulation
- **Module 3: Isaac Platform** - NVIDIA Isaac Sim, Isaac ROS, GPU-accelerated perception
- **Module 4: VLA & Humanoids** - Vision-Language-Action models and humanoid deployment

### 🤖 AI-Powered Features
- **RAG Chatbot** - Context-aware AI assistant powered by OpenAI and Qdrant vector search
- **Text Selection "Ask AI"** - Select any text and ask the AI for clarification
- **Personalized Content** - Content adapted to your experience level and background
- **Multi-Language Translation** - Translate content to Urdu (expandable to other languages)

### 👤 User Management
- **Authentication** - Secure signup/login with JWT tokens
- **User Profiles** - Track experience level, interests, and learning goals
- **Onboarding Flow** - Personalized setup for new users

### 🎨 Premium UI/UX
- **Glassmorphism Design** - Modern, elegant frosted-glass aesthetic
- **Dark Mode Optimized** - Deep space backgrounds with electric violet accents
- **Smooth Animations** - Fade-ins, micro-interactions,and responsive transitions
- **Mobile Responsive** - Works beautifully on all devices

## 🏗️ Architecture

### Frontend (Docusaurus + React + TypeScript)
```
src/
├── components/          # React components
│   ├── Auth/           # Authentication modal
│   ├── ChatBot/        # RAG-powered chat interface
│   ├── Onboarding/     # User onboarding flow
│   ├── PersonalizeButton/  # Content personalization
│   ├── TranslateButton/    # Multi-language translation
│   └── TextSelection/      # "Ask AI" text selection handler
├── css/                # Global styles with premium theme
├── pages/              # Landing page and custom routes
└── theme/              # Docusaurus theme customizations
```

### Backend (FastAPI + Python)
```
backend/
├── routes/             # API endpoints
│   ├── auth.py        # Authentication (signup/login/tokens)
│   ├── profile.py     # User profile management
│   ├── chat.py        # RAG chatbot queries
│   ├── personalize.py # Content personalization
│   └── translate.py   # Translation service
├── rag/                # RAG implementation
│   ├── ingestion.py   # Vector DB ingestion pipeline
│   └── query.py       # RAG query engine
├── database.py         # SQLAlchemy models (Postgres/Neon)
├── config.py           # Environment configuration
└── main.py             # FastAPI application entry point
```

## 🚀 Getting Started

### Prerequisites
- **Node.js** 20.0 or higher
- **Python** 3.10 or higher
- **PostgreSQL** (or Neon serverless Postgres)
- **Qdrant** (cloud or local vector database)
- **OpenAI API Key**

### Frontend Setup

1. **Install dependencies:**
   ```bash
   npm install
   ```

2. **Start development server:**
   ```bash
   npm start
   ```
   This opens `http://localhost:3000`

3. **Build for production:**
   ```bash
   npm run build
   ```

4. **Serve build locally:**
   ```bash
   npm run serve
   ```

### Backend Setup

1. **Navigate to backend:**
   ```bash
   cd backend
   ```

2. **Create virtual environment:**
   ```bash
   python -m venv venv
   
   # Windows
   .\\venv\\Scripts\\activate
   
   # macOS/Linux
   source venv/bin/activate
   ```

3. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Configure environment:**
   - Copy `.env.example` to `.env`
   - Fill in your credentials:
     ```env
     OPENAI_API_KEY=your_openai_key
     DATABASE_URL=your_postgres_url
     QDRANT_URL=your_qdrant_url
     QDRANT_API_KEY=your_qdrant_key
     JWT_SECRET=your_random_secret
     ```

5. **Run ingestion (first time):**
   ```bash
   python -m rag.ingestion
   ```
   This indexes all documentation into the vector database.

6. **Start backend server:**
   ```bash
   uvicorn main:app --reload --port 8000
   ```
   API docs available at `http://localhost:8000/docs`

## 🗂️ Database Schema

### User Table
- `id` - Primary key
- `email` - Unique email address
- `hashed_password` - Bcrypt hashed password
- `created_at` - Registration timestamp

### UserProfile Table
- `id` - Primary key
- `user_id` - Foreign key to User
- `role` - student | researcher | hobbyist | professional
- `experience_level` - beginner | intermediate | advanced
- `interests` - JSON array of topic interests
- `software_experience` - Software background level
- `hardware_experience` - Hardware background level

## 🔌 API Endpoints

### Authentication
- `POST /api/auth/signup` - Create new user
- `POST /api/auth/token` - Login (returns JWT)
- `GET /api/auth/me` - Get current user

### Profile
- `GET /api/profile/` - Get user profile
- `PUT /api/profile/` - Update profile

### AI Features
- `POST /api/chat/` - RAG chatbot query
- `POST /api/personalize/` - Personalize content
- `POST /api/translate/` - Translate content

## 🎨 Design System

### Color Palette
- **Primary:** Electric Violet (`#7c3aed`)
- **Background (Dark):** Deep Space (`#030712`)
- **Surface:** Dark Gray (`#111827`)
- **Accent:** Cyber Blue gradient

### Typography
- **Font Family:** Inter (Google Fonts fallback system)
- **Headings:** 800 weight, gradient text effects
- **Body:** Clean, readable spacing

### Components
- **Glass Panels:** `backdrop-filter: blur(12px)`, semi-transparent backgrounds
- **Buttons:** Gradient fills, shadow effects, hover animations
- **Modals:** Centered overlays with backdrop blur

## 📦 Dependencies

### Frontend
- `@docusaurus/core` - Static site generator
- `react` 19.0 - UI library
- `react-markdown` - Markdown rendering
- `typescript` - Type safety

### Backend
- `fastapi` - Modern Python web framework
- `uvicorn` - ASGI server
- `sqlalchemy` - ORM with async support
- `openai` - OpenAI API client
- `qdrant-client` - Vector database client
- `pyjwt` - JWT authentication
- `bcrypt` - Password hashing

## 🚢 Deployment

### Frontend (Vercel / GitHub Pages)
```bash
npm run build
npm run deploy  # If using GitHub Pages
```

For Vercel:
1. Connect your repository
2. Build command: `npm run build`
3. Output directory: `build`

### Backend (Railway / Render / Fly.io)
1. Set environment variables in your platform
2. Use `uvicorn main:app --host 0.0.0.0 --port $PORT`
3. Ensure PostgreSQL and Qdrant are accessible

## 🧪 Testing

### Manual Testing Checklist
- [ ] Landing page loads with animations
- [ ] Navigate to documentation modules
- [ ] Sign up new user
- [ ] Log in existing user
- [ ] Complete onboarding flow
- [ ] Open chatbot and ask questions
- [ ] Select text and use "Ask AI"
- [ ] Personalize content (requires login)
- [ ] Translate content to Urdu
- [ ] Verify responsive design on mobile

## 📝 Content Management

Documentation is stored in `docs/` as Markdown files:
- Edit existing chapters in their respective module folders
- Add new chapters and update `sidebars.ts`
- Re-run ingestion after significant content changes

## 🛠️ Development Workflow

1. **Frontend Development:**
   - Edit React components in `src/components/`
   - Update styles in component CSS modules or `src/css/custom.css`
   - Test with `npm start`

2. **Backend Development:**
   - Add/modify routes in `backend/routes/`
   - Test endpoints at `http://localhost:8000/docs`
   - Re-run ingestion if RAG data changes

3. **Content Updates:**
   - Edit Markdown in `docs/`
   - Rebuild frontend to see changes
   - Re-ingest for RAG updates

## 🐛 Troubleshooting

### Frontend Issues
- **Build fails:** Make sure all CSS modules exist for components that import them
- **Components not rendering:** Check console for TypeScript errors
- **Styles not applying:** Verify CSS module import paths

### Backend Issues
- **Database connection:** Verify `DATABASE_URL` in `.env`
- **Qdrant errors:** Check `QDRANT_URL` and `QDRANT_API_KEY`
- **OpenAI errors:** Verify `OPENAI_API_KEY` is valid
- **CORS errors:** Ensure frontend URL is in CORS allowed origins in `main.py`

### RAG Not Working
1. Verify Qdrant collection exists: Check Qdrant dashboard
2. Re-run ingestion: `python -m rag.ingestion`
3. Test query endpoint directly via `/docs`

## 📄 License

This project is built for educational purposes as part of a hackathon submission.

## 🙏 Acknowledgments

- **Docusaurus** - Amazing documentation framework
- **OpenAI** - GPT models and embeddings
- **Qdrant** - High-performance vector search
- **Neon** - Serverless Postgres

## 🤝 Contributing

This is a hackathon project, but suggestions and improvements are welcome! Feel free to:
- Report issues
- Suggest new features
- Improve documentation
- Enhance UI/UX

## 📧 Contact

For questions or feedback about this project, please open an issue in the repository.

---

**Built with ❤️ for the future of Physical AI and Robotics education**
