/**
 * Type definitions for ChatBot component.
 */

import type { Citation } from '../../services/api';

/**
 * A single message in the chat.
 */
export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  citations?: Citation[];
  isLoading?: boolean;
  error?: string;
}

/**
 * Chat state for the chatbot.
 */
export interface ChatState {
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
  isOpen: boolean;
}

/**
 * Props for the ChatBot component.
 */
export interface ChatBotProps {
  /** Initial open state */
  defaultOpen?: boolean;
  /** Callback when chat is opened/closed */
  onToggle?: (isOpen: boolean) => void;
  /** Custom class name */
  className?: string;
}

/**
 * Props for the ChatMessage component.
 */
export interface ChatMessageProps {
  message: ChatMessage;
  onCitationClick?: (citation: Citation) => void;
}

/**
 * Props for the ChatInput component.
 */
export interface ChatInputProps {
  onSubmit: (query: string) => void;
  disabled?: boolean;
  placeholder?: string;
}

/**
 * Props for the CitationList component.
 */
export interface CitationListProps {
  citations: Citation[];
  onCitationClick?: (citation: Citation) => void;
}

/**
 * Context for selected text functionality.
 */
export interface SelectedTextContext {
  text: string;
  chapter?: string;
  section?: string;
}
