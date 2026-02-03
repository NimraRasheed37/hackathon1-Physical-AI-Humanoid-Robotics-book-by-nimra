/**
 * ChatInput component for user message input.
 */

import React, { useState, useCallback, useRef, useEffect } from 'react';
import type { ChatInputProps } from './types';
import { config } from '../../services/config';
import styles from './ChatBot.module.css';

export function ChatInput({
  onSubmit,
  disabled = false,
  placeholder = config.placeholderText,
}: ChatInputProps): JSX.Element {
  const [query, setQuery] = useState('');
  const inputRef = useRef<HTMLInputElement>(null);

  // Focus input when not disabled
  useEffect(() => {
    if (!disabled && inputRef.current) {
      inputRef.current.focus();
    }
  }, [disabled]);

  const handleSubmit = useCallback(
    (e: React.FormEvent) => {
      e.preventDefault();
      const trimmedQuery = query.trim();
      if (trimmedQuery && !disabled) {
        onSubmit(trimmedQuery);
        setQuery('');
      }
    },
    [query, disabled, onSubmit]
  );

  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent) => {
      if (e.key === 'Enter' && !e.shiftKey) {
        handleSubmit(e);
      }
    },
    [handleSubmit]
  );

  return (
    <div className={styles.inputArea}>
      <form className={styles.inputForm} onSubmit={handleSubmit}>
        <input
          ref={inputRef}
          type="text"
          className={styles.input}
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={placeholder}
          disabled={disabled}
          maxLength={config.maxQueryLength}
          aria-label="Chat message input"
        />
        <button
          type="submit"
          className={styles.sendButton}
          disabled={disabled || !query.trim()}
          aria-label="Send message"
        >
          <span className={styles.sendIcon}>➤</span>
        </button>
      </form>
    </div>
  );
}
