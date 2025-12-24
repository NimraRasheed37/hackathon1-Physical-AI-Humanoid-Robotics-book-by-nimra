# Quickstart: AI-Native Book

This guide explains how to set up and run the Docusaurus project for the AI-Native Book locally.

## Prerequisites

-   [Node.js](https://nodejs.org/en/) (version 18.0 or later)
-   [Yarn](https://yarnpkg.com/) (or npm, pnpm)

## Setup

1.  **Initialize the Docusaurus project**:
    If the project has not been set up yet, initialize a new Docusaurus site.

    ```bash
    npx create-docusaurus@latest . classic
    ```

2.  **Install dependencies**:
    Install the project dependencies using yarn (or your preferred package manager).

    ```bash
    yarn install
    ```

3.  **Project Structure**:
    The book content is located in the `/docs` directory. The structure is as follows:

    ```
    /docs
      /book
        /module-1-ros2-robotic-nervous-system
          chapter-1-ros2-fundamentals.md
          chapter-2-ros2-communication-control.md
          chapter-3-urdf-humanoid-modeling.md
    ```

## Running Locally

1.  **Start the development server**:
    Run the following command to start the Docusaurus development server.

    ```bash
    yarn start
    ```

2.  **View the site**:
    Open your browser and navigate to `http://localhost:3000` to see the site in action. The development server provides live reloading, so any changes you make to the content will be reflected automatically.

## Building the Site

To create a static build of the site for production, run the following command:

```bash
yarn build
```

The built files will be located in the `/build` directory. You can test the build locally by running `yarn serve`.
