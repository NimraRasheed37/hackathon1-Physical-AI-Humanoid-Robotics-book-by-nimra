/**
 * API client for RAG Chatbot backend.
 */

import { config, getApiUrl, getSessionId } from './config';

/**
 * Citation information from a source chunk.
 */
export interface Citation {
  chunk_id: string;
  content: string;
  chapter: string;
  section: string | null;
  similarity_score: number;
}

/**
 * Chat response from the API.
 */
export interface ChatResponse {
  answer: string;
  citations: Citation[];
  session_id: string;
  timestamp: string;
  model_used: string;
}

/**
 * Error response from the API.
 */
export interface ApiError {
  error: string;
  message: string;
  details?: Record<string, unknown>;
}

/**
 * Chat request payload.
 */
interface ChatRequest {
  query: string;
  session_id?: string;
}

/**
 * Chat with selected text request payload.
 */
interface SelectedTextChatRequest {
  query: string;
  selected_text: string;
  context_metadata?: Record<string, unknown>;
  session_id?: string;
}

/**
 * API client class for chatbot interactions.
 */
class ChatApiClient {
  private abortController: AbortController | null = null;

  /**
   * Send a chat query to the API.
   */
  async chat(query: string): Promise<ChatResponse> {
    // Cancel any pending request
    this.cancelPendingRequest();

    this.abortController = new AbortController();

    const payload: ChatRequest = {
      query: query.slice(0, config.maxQueryLength),
      session_id: getSessionId(),
    };

    const response = await fetch(getApiUrl('/chat'), {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
      signal: this.abortController.signal,
    });

    if (!response.ok) {
      const error: ApiError = await response.json();
      throw new Error(error.message || 'Failed to send message');
    }

    return response.json();
  }

  /**
   * Send a chat query with selected text context.
   */
  async chatWithSelectedText(
    query: string,
    selectedText: string,
    contextMetadata?: Record<string, unknown>
  ): Promise<ChatResponse> {
    // Cancel any pending request
    this.cancelPendingRequest();

    this.abortController = new AbortController();

    const payload: SelectedTextChatRequest = {
      query: query.slice(0, config.maxQueryLength),
      selected_text: selectedText.slice(0, config.maxSelectedTextLength),
      context_metadata: contextMetadata,
      session_id: getSessionId(),
    };

    const response = await fetch(getApiUrl('/chat/selected'), {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
      signal: this.abortController.signal,
    });

    if (!response.ok) {
      const error: ApiError = await response.json();
      throw new Error(error.message || 'Failed to send message');
    }

    return response.json();
  }

  /**
   * Check API health status.
   */
  async checkHealth(): Promise<boolean> {
    try {
      const response = await fetch(`${config.apiBaseUrl}/health`, {
        method: 'GET',
      });
      return response.ok;
    } catch {
      return false;
    }
  }

  /**
   * Cancel any pending request.
   */
  cancelPendingRequest(): void {
    if (this.abortController) {
      this.abortController.abort();
      this.abortController = null;
    }
  }
}

// Export singleton instance
export const chatApi = new ChatApiClient();
