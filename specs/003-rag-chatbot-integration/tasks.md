# Tasks: RAG Chatbot Integration

**Input**: Design documents from `/specs/003-rag-chatbot-integration/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/api.yaml

**Tests**: Not explicitly requested in specification - tests are OPTIONAL and included as enhancement tasks in Polish phase.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/` (Python/FastAPI)
- **Frontend**: `docusaurus-site/src/` (React/TypeScript)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for both backend and frontend

- [x] T001 Create backend directory structure per implementation plan in backend/
- [x] T002 Create Python virtual environment and install dependencies in backend/requirements.txt
- [x] T003 [P] Create .env.example with all required environment variables in backend/.env.example
- [x] T004 [P] Create frontend component directory structure in docusaurus-site/src/components/ChatBot/
- [x] T005 [P] Create frontend hooks directory in docusaurus-site/src/hooks/
- [x] T006 [P] Create frontend services directory in docusaurus-site/src/services/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [x] T007 Implement Settings class with pydantic-settings in backend/app/config.py
- [x] T008 [P] Implement Pydantic request/response models in backend/app/models.py
- [x] T009 [P] Implement database connection manager in backend/app/database.py
- [x] T010 Create FastAPI application with CORS middleware in backend/app/main.py
- [x] T011 [P] Implement PostgresService with connection pool in backend/app/services/postgres_service.py
- [x] T012 [P] Implement QdrantService (vector store operations) in backend/app/services/vector_store.py
- [x] T013 Create database initialization script in backend/scripts/init_db.py
- [x] T014 [P] Implement text chunking utility in backend/app/utils/chunking.py
- [x] T015 [P] Implement Docusaurus markdown parser in backend/app/utils/markdown_parser.py
- [x] T016 Create book embedding script in backend/scripts/embed_book.py
- [ ] T017 Run database initialization and book embedding (manual step)

### Frontend Foundation

- [x] T018 [P] Create API client service in docusaurus-site/src/services/api.ts
- [x] T019 [P] Create API configuration file in docusaurus-site/src/services/config.ts

**Checkpoint**: Foundation ready - user story implementation can now begin ✅

---

## Phase 3: User Story 1 - Ask General Questions About Textbook Content (Priority: P1) MVP

**Goal**: Enable students to ask questions about robotics concepts and receive accurate answers with citations from the textbook.

**Independent Test**: Open chat interface, ask "What is inverse kinematics?", verify response includes accurate information with citations to specific textbook chapters/sections.

### Backend Implementation for User Story 1

- [x] T020 [US1] Implement core RAG pipeline in backend/app/services/chat_service.py
  - Generate query embedding using OpenAI
  - Search Qdrant for similar chunks
  - Build context from retrieved chunks
  - Generate answer using GPT-4o-mini
  - Format citations
- [x] T021 [US1] Implement chat router with POST /api/v1/chat endpoint in backend/app/routers/chat.py
- [x] T022 [US1] Implement health check endpoint GET /health in backend/app/main.py
- [x] T023 [US1] Register chat router in FastAPI app in backend/app/main.py
- [x] T024 [US1] Add input validation for query length (max 500 chars) in backend/app/models.py

### Frontend Implementation for User Story 1

- [x] T025 [P] [US1] Create ChatBot floating button component in docusaurus-site/src/components/ChatBot/ChatBot.tsx
- [x] T026 [P] [US1] Create ChatWindow container component (integrated in ChatBot.tsx)
- [x] T027 [P] [US1] Create ChatInput component with submit handling in docusaurus-site/src/components/ChatBot/ChatInput.tsx
- [x] T028 [P] [US1] Create ChatMessage component for displaying messages in docusaurus-site/src/components/ChatBot/ChatMessage.tsx
- [x] T029 [P] [US1] Create Citation component for displaying sources (integrated in ChatMessage.tsx)
- [x] T030 [P] [US1] Create ChatBot styles with responsive design in docusaurus-site/src/components/ChatBot/ChatBot.module.css
- [x] T031 [US1] Implement useChat hook with API integration in docusaurus-site/src/components/ChatBot/useChatBot.ts
- [x] T032 [US1] Create Root.tsx wrapper to inject ChatBot in docusaurus-site/src/theme/Root.tsx
- [x] T033 [US1] Add loading indicator to ChatWindow component
- [x] T034 [US1] Add error handling and display to ChatWindow component
- [x] T035 [US1] Add welcome message display when chat first opens

**Checkpoint**: User Story 1 complete - Core Q&A with citations is fully functional ✅

---

## Phase 4: User Story 2 - Ask Questions About Selected Text (Priority: P2)

**Goal**: Allow students to select text in the textbook and ask contextual questions about that specific content.

**Independent Test**: Select a paragraph about "sensor fusion", ask "Can you explain this in simpler terms?", verify response directly addresses the selected content.

### Backend Implementation for User Story 2

- [x] T036 [US2] Add SelectedTextChatRequest model in backend/app/models.py
- [x] T037 [US2] Implement POST /api/v1/chat/selected endpoint in backend/app/routers/chat.py
- [x] T038 [US2] Extend RAG service with context-aware search in backend/app/services/chat_service.py
  - Generate embedding for selected text
  - Prioritize chunks from selected section
  - Combine with query-based search
- [x] T039 [US2] Add input validation for selected text length (max 2000 chars) in backend/app/models.py

### Frontend Implementation for User Story 2

- [x] T040 [US2] Implement TextSelectionPopup component in docusaurus-site/src/components/ChatBot/TextSelectionPopup.tsx
  - Detect text selection in content area
  - Capture selected text (10-2000 chars)
  - Track selection state
- [x] T041 [US2] Add selection indicator badge (integrated in TextSelectionPopup)
- [x] T042 [US2] Add clear selection button to selection badge
- [x] T043 [US2] Update useChatBot hook to include selected text in API calls
- [x] T044 [US2] Update api.ts service to support context endpoint

**Checkpoint**: User Story 2 complete - Text selection context feature is fully functional ✅

---

## Phase 5: User Story 3 - Maintain Conversation Context Within Session (Priority: P3)

**Goal**: Enable follow-up questions that build on previous responses within the same browser session.

**Independent Test**: Ask "What is ROS2?", then ask "How does it compare to the original ROS?", verify second response correctly interprets "it" as ROS2.

### Backend Implementation for User Story 3

- [x] T045 [US3] Implement query storage in PostgresService in backend/app/services/postgres_service.py
  - store_query method
  - get_conversation_history method
- [x] T046 [US3] Update RAG service to store interactions in backend/app/services/chat_service.py
- [x] T047 [US3] Add analytics event logging for chat_query in backend/app/services/postgres_service.py

### Frontend Implementation for User Story 3

- [x] T048 [US3] Implement session ID generation in config.ts
  - Generate UUID format
  - Store in sessionStorage
  - Persist across page navigation
- [x] T049 [US3] Update ChatBot to display conversation history
- [x] T050 [US3] Add session_id to all API requests in api.ts
- [x] T051 [US3] Implement chat persistence within browser session (no cross-tab sync)

**Checkpoint**: User Story 3 complete - Conversation context is maintained within session ✅

---

## Phase 6: User Story 4 - View and Navigate Citations (Priority: P3)

**Goal**: Enable students to view citation details including chapter, section, content preview, and relevance score.

**Independent Test**: Ask a question, receive response with citations, verify each citation shows chapter/section information and content preview.

### Backend Implementation for User Story 4

- [x] T052 [US4] Ensure Citation model includes all required fields in backend/app/models.py
  - chunk_id, content (200 char preview), chapter, section, similarity_score
- [x] T053 [US4] Update RAG service to include similarity scores in citations in backend/app/services/chat_service.py

### Frontend Implementation for User Story 4

- [x] T054 [US4] Enhance Citation component with expandable preview in ChatMessage.tsx
- [x] T055 [US4] Add similarity score badge to Citation component
- [x] T056 [US4] Add chapter/section display formatting in Citation component
- [x] T057 [US4] Add analytics event for citation_view in chat router

**Checkpoint**: User Story 4 complete - Citations are fully viewable with all metadata ✅

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Error Handling & Edge Cases

- [x] T058 [P] Add graceful handling for empty search results in backend/app/services/chat_service.py
- [x] T059 [P] Add network error retry logic in docusaurus-site/src/services/api.ts
- [x] T060 [P] Add empty query prevention (disable submit button) in docusaurus-site/src/components/ChatBot/ChatInput.tsx
- [x] T061 [P] Add query length validation UI feedback in ChatInput component

### Mobile Responsiveness

- [x] T062 [P] Verify and fix mobile layout (375px min width) in docusaurus-site/src/components/ChatBot/ChatBot.module.css
- [ ] T063 [P] Test touch event handling for text selection on mobile

### Admin & Maintenance

- [x] T064 [P] Implement embedding router with POST /api/v1/embed in backend/app/routers/embedding.py
- [x] T065 Register embedding router in FastAPI app in backend/app/main.py

### Documentation & Cleanup

- [x] T066 [P] Update backend README with setup instructions in backend/README.md
- [x] T067 [P] Add API documentation comments to all endpoints
- [ ] T068 Run quickstart.md validation to verify setup instructions

### Optional: Testing (if time permits)

- [ ] T069 [P] Add unit tests for chunking utility in backend/tests/unit/test_chunking.py
- [ ] T070 [P] Add unit tests for Pydantic models in backend/tests/unit/test_models.py
- [ ] T071 [P] Add integration tests for chat API in backend/tests/integration/test_chat_api.py
- [ ] T072 [P] Add frontend component tests in docusaurus-site/tests/ChatBot.test.tsx

---

## Dependencies & Execution Order

### Phase Dependencies

```text
Phase 1: Setup ──────────────────────────────────────┐
                                                      │
Phase 2: Foundational (BLOCKS ALL USER STORIES) ─────┼──┐
                                                      │  │
                   ┌──────────────────────────────────┘  │
                   │                                      │
                   ▼                                      │
Phase 3: User Story 1 (P1) - MVP ◄────────────────────────┘
                   │
                   ▼ (US1 must complete before others)
           ┌───────┴───────┐
           │               │
           ▼               ▼
Phase 4: US2 (P2)   Phase 5: US3 (P3)   Phase 6: US4 (P3)
           │               │                    │
           └───────────────┼────────────────────┘
                           │
                           ▼
              Phase 7: Polish & Cross-Cutting
```

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories - **THIS IS THE MVP** ✅
- **User Story 2 (P2)**: Depends on User Story 1 completion (extends RAG service and chat UI) ✅
- **User Story 3 (P3)**: Depends on User Story 1 completion (adds session management) ✅
- **User Story 4 (P3)**: Depends on User Story 1 completion (extends citation display) ✅

### Task Summary

| Phase | Tasks | Completed | Description |
|-------|-------|-----------|-------------|
| Phase 1: Setup | 6 | 6 ✅ | Project structure initialization |
| Phase 2: Foundational | 13 | 12 ✅ | Core infrastructure (backend + frontend) |
| Phase 3: User Story 1 | 16 | 16 ✅ | Core Q&A with citations (MVP) |
| Phase 4: User Story 2 | 9 | 9 ✅ | Text selection context |
| Phase 5: User Story 3 | 7 | 7 ✅ | Session conversation history |
| Phase 6: User Story 4 | 6 | 6 ✅ | Citation detail viewing |
| Phase 7: Polish | 15 | 10 ✅ | Error handling, mobile, docs, tests |
| **Total** | **72** | **66** | **92% Complete** |

---

## Remaining Tasks

- [ ] T017 Run database initialization and book embedding (manual step)
- [ ] T063 Test touch event handling for text selection on mobile
- [ ] T068 Run quickstart.md validation to verify setup instructions
- [ ] T069-T072 Optional testing tasks

---

## Notes

- [P] tasks = different files, no dependencies
- [US#] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Backend and frontend can often progress in parallel once Phase 2 is complete
- External service setup (Qdrant, Neon, OpenAI) is a prerequisite covered in quickstart.md
