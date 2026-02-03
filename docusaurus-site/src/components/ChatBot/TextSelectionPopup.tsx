/**
 * TextSelectionPopup component for quick actions on selected text.
 */

import React, { useEffect, useState, useCallback } from 'react';
import styles from './TextSelectionPopup.module.css';

interface PopupPosition {
  top: number;
  left: number;
}

interface TextSelectionPopupProps {
  onExplain: (text: string) => void;
  onAsk: (text: string) => void;
  minSelectionLength?: number;
  maxSelectionLength?: number;
}

export function TextSelectionPopup({
  onExplain,
  onAsk,
  minSelectionLength = 10,
  maxSelectionLength = 2000,
}: TextSelectionPopupProps): JSX.Element | null {
  const [selectedText, setSelectedText] = useState<string>('');
  const [position, setPosition] = useState<PopupPosition | null>(null);
  const [isVisible, setIsVisible] = useState(false);

  const handleSelectionChange = useCallback(() => {
    const selection = window.getSelection();

    if (!selection || selection.isCollapsed) {
      setIsVisible(false);
      setSelectedText('');
      return;
    }

    const text = selection.toString().trim();

    // Check if selection is valid
    if (text.length < minSelectionLength || text.length > maxSelectionLength) {
      setIsVisible(false);
      setSelectedText('');
      return;
    }

    // Check if selection is in document content (not in chat widget)
    const range = selection.getRangeAt(0);
    const container = range.commonAncestorContainer;
    const element = container.nodeType === Node.TEXT_NODE
      ? container.parentElement
      : container as Element;

    // Don't show popup for selections in the chat widget
    if (element?.closest('[class*="chatBot"]')) {
      setIsVisible(false);
      return;
    }

    // Check if selection is in main content area
    const isInContent = element?.closest('article') ||
                       element?.closest('.markdown') ||
                       element?.closest('[class*="docMainContainer"]');

    if (!isInContent) {
      setIsVisible(false);
      return;
    }

    // Get selection position
    const rect = range.getBoundingClientRect();
    const popupTop = rect.bottom + window.scrollY + 8;
    const popupLeft = Math.max(
      10,
      Math.min(
        rect.left + rect.width / 2 - 100,
        window.innerWidth - 220
      )
    );

    setSelectedText(text);
    setPosition({ top: popupTop, left: popupLeft });
    setIsVisible(true);
  }, [minSelectionLength, maxSelectionLength]);

  const handleExplain = useCallback(() => {
    if (selectedText) {
      onExplain(selectedText);
      window.getSelection()?.removeAllRanges();
      setIsVisible(false);
    }
  }, [selectedText, onExplain]);

  const handleAsk = useCallback(() => {
    if (selectedText) {
      onAsk(selectedText);
      window.getSelection()?.removeAllRanges();
      setIsVisible(false);
    }
  }, [selectedText, onAsk]);

  // Listen for selection changes
  useEffect(() => {
    const handleMouseUp = () => {
      // Small delay to ensure selection is complete
      setTimeout(handleSelectionChange, 10);
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      if (e.key === 'Shift') {
        setTimeout(handleSelectionChange, 10);
      }
    };

    const handleClickOutside = (e: MouseEvent) => {
      const target = e.target as Element;
      if (!target.closest('[class*="popup"]')) {
        setIsVisible(false);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('keyup', handleKeyUp);
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('keyup', handleKeyUp);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [handleSelectionChange]);

  if (!isVisible || !position) {
    return null;
  }

  return (
    <div
      className={styles.popup}
      style={{
        top: position.top,
        left: position.left,
      }}
    >
      <button
        className={`${styles.button} ${styles.explainButton}`}
        onClick={handleExplain}
        title="Ask the AI to explain this text"
      >
        <span className={styles.buttonIcon}>💡</span>
        Explain
      </button>
      <button
        className={`${styles.button} ${styles.askButton}`}
        onClick={handleAsk}
        title="Ask a question about this text"
      >
        <span className={styles.buttonIcon}>❓</span>
        Ask about
      </button>
    </div>
  );
}
