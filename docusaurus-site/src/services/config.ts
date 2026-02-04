/**
 * Frontend configuration for RAG Chatbot.
 */

// Get API URL from window (set by Root.tsx from Docusaurus customFields)
function getApiBaseUrl(): string {
  if (typeof window !== 'undefined' && (window as any).__RAG_API_BASE_URL__) {
    return (window as any).__RAG_API_BASE_URL__;
  }
  return 'http://localhost:8001';
}

export const config = {
  // API Configuration
  apiBaseUrl: getApiBaseUrl(),
  apiVersion: 'v1',

  // Chat Configuration
  maxQueryLength: 500,
  maxSelectedTextLength: 2000,
  sessionStorageKey: 'rag_chatbot_session_id',

  // UI Configuration
  chatBotTitle: 'AI Assistant',
  chatBotSubtitle: 'Ask questions about the textbook',
  placeholderText: 'Ask a question about the book...',

  // Timeouts
  requestTimeoutMs: 30000,
} as const;

/**
 * Get the full API URL for a given endpoint.
 */
export function getApiUrl(endpoint: string): string {
  const base = config.apiBaseUrl.replace(/\/$/, '');
  const path = endpoint.startsWith('/') ? endpoint : `/${endpoint}`;
  return `${base}/api/${config.apiVersion}${path}`;
}

/**
 * Get or create a session ID for the current user.
 */
export function getSessionId(): string {
  if (typeof window === 'undefined') {
    return '';
  }

  let sessionId = sessionStorage.getItem(config.sessionStorageKey);

  if (!sessionId) {
    sessionId = crypto.randomUUID();
    sessionStorage.setItem(config.sessionStorageKey, sessionId);
  }

  return sessionId;
}

/**
 * Clear the current session.
 */
export function clearSession(): void {
  if (typeof window !== 'undefined') {
    sessionStorage.removeItem(config.sessionStorageKey);
  }
}
