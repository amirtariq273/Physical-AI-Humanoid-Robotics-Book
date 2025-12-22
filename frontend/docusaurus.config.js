// @ts-check
const { themes } = require('prism-react-renderer');
const lightCodeTheme = themes.github;
const darkCodeTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline:
    'Building the Future of Intelligent Robots, from foundational concepts to advanced, real-world applications.',
  url: 'https://physical-ai-humanoid-robotics-book-nu-ruby.vercel.app', // ✅ Correct deployment URL
  baseUrl: '/',
  onBrokenLinks: 'warn',

  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  favicon: 'img/book-logo.png',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  stylesheets: [
    'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css',
    '/css/globals.css',
  ],

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/amirtariq273/Physical-AI-Humanoid-Robotics/tree/main/frontend/',
        },
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/amirtariq273/Physical-AI-Humanoid-Robotics/tree/main/frontend/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: false,
      },
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'My Site Logo',
          src: 'img/book-logo.png',
        },
        items: [
          {
            type: 'doc',
            docId: 'ros2-nervous-system/chapter1_basics',
            position: 'left',
            label: 'Modules',
            className: 'modules-link-bold',
          },
          {
            href: 'https://github.com/amirtariq273/Physical-AI-Humanoid-Robotics',
            label: 'GitHub',
            position: 'right',
          },
          {
            to: '/signup',
            label: 'Sign Up',
            position: 'right',
            className: 'button button--primary button--signup',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'ROS 2 Nervous System',
                to: '/docs/ros2-nervous-system/chapter1_basics',
              },
              {
                label: 'Digital Twin Simulation',
                to: '/docs/digital-twin-sim/chapter1_gazebo_physics',
              },
              {
                label: 'AI-Robot Brain (Isaac Sim)',
                to: '/docs/isaac-robot-brain/chapter1_isaac_sim',
              },
              {
                label: 'Vision-Language-Action (VLA)',
                to: '/docs/vla-humanoid-robotics/chapter1_whisper',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/robotics',
              },
              {
                label: 'Discord',
                href: 'https://discord.com/invite/robotics',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/robotics',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/amirtariq273/Physical-AI-Humanoid-Robotics',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Project. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;
