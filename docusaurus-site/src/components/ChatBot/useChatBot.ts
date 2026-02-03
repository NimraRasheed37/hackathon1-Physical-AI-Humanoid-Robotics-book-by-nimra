/**
 * Custom hook for ChatBot state management.
 */

import { useState, useCallback, useRef } from 'react';
import { chatApi, type ChatResponse, type Citation } from '../../services/api';
import type { ChatMessage, ChatState, SelectedTextContext } from './types';

function generateMessageId(): string {
  return `msg_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

interface UseChatBotReturn {
  state: ChatState;
  sendMessage: (query: string, selectedText?: SelectedTextContext) => Promise<void>;
  clearMessages: () => void;
  toggleOpen: () => void;
  setOpen: (isOpen: boolean) => void;
}

export function useChatBot(defaultOpen = false): UseChatBotReturn {
  const [state, setState] = useState<ChatState>({
    messages: [],
    isLoading: false,
    error: null,
    isOpen: defaultOpen,
  });

  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = useCallback(() => {
    setTimeout(() => {
      messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, 100);
  }, []);

  const sendMessage = useCallback(
    async (query: string, selectedText?: SelectedTextContext) => {
      // Create user message
      const userMessage: ChatMessage = {
        id: generateMessageId(),
        role: 'user',
        content: query,
        timestamp: new Date(),
      };

      // Create placeholder assistant message
      const assistantMessageId = generateMessageId();
      const assistantMessage: ChatMessage = {
        id: assistantMessageId,
        role: 'assistant',
        content: '',
        timestamp: new Date(),
        isLoading: true,
      };

      // Update state with both messages
      setState((prev) => ({
        ...prev,
        messages: [...prev.messages, userMessage, assistantMessage],
        isLoading: true,
        error: null,
      }));

      scrollToBottom();

      try {
        let response: ChatResponse;

        if (selectedText?.text) {
          response = await chatApi.chatWithSelectedText(
            query,
            selectedText.text,
            {
              chapter: selectedText.chapter,
              section: selectedText.section,
            }
          );
        } else {
          response = await chatApi.chat(query);
        }

        // Update assistant message with response
        setState((prev) => ({
          ...prev,
          messages: prev.messages.map((msg) =>
            msg.id === assistantMessageId
              ? {
                  ...msg,
                  content: response.answer,
                  citations: response.citations,
                  isLoading: false,
                }
              : msg
          ),
          isLoading: false,
        }));

        scrollToBottom();
      } catch (error) {
        const errorMessage = error instanceof Error ? error.message : 'An error occurred';

        // Update assistant message with error
        setState((prev) => ({
          ...prev,
          messages: prev.messages.map((msg) =>
            msg.id === assistantMessageId
              ? {
                  ...msg,
                  content: 'Sorry, I encountered an error. Please try again.',
                  error: errorMessage,
                  isLoading: false,
                }
              : msg
          ),
          isLoading: false,
          error: errorMessage,
        }));
      }
    },
    [scrollToBottom]
  );

  const clearMessages = useCallback(() => {
    setState((prev) => ({
      ...prev,
      messages: [],
      error: null,
    }));
  }, []);

  const toggleOpen = useCallback(() => {
    setState((prev) => ({
      ...prev,
      isOpen: !prev.isOpen,
    }));
  }, []);

  const setOpen = useCallback((isOpen: boolean) => {
    setState((prev) => ({
      ...prev,
      isOpen,
    }));
  }, []);

  return {
    state,
    sendMessage,
    clearMessages,
    toggleOpen,
    setOpen,
  };
}
