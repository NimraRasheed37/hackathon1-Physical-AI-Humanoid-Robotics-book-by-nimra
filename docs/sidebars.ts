import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'book-intro',
    {
      type: 'category',
      label: 'Module 1: ROS Fundamentals',
      items: [
        'module-1-ros-fundamentals',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Basics',
      items: [
        'module-2-simulation-basics',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Advanced Simulation',
      items: [
        'module-3-advanced-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA and Planning',
      items: [
        'module-4-vla-and-planning',
      ],
    },
    'capstone-project',
    'glossary',
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
