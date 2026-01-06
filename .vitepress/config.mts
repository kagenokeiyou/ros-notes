import { defineConfig } from 'vitepress'

const base = '/ros-notes/'

export default defineConfig({
  title: 'ROS Notes',
  description: 'ROS notes written by keiyou',
  lang: 'zh-CN',
  base: base,
  head: [['link', { rel: 'icon', href: base + 'avatar.png' }]],
  appearance: 'dark',
  cleanUrls: true,
  lastUpdated: true,
  themeConfig: {
    logo: '/avatar.png',
    search: {
      provider: 'local',
    },
    socialLinks: [{ icon: 'github', link: 'https://github.com/kagenokeiyou/ros-notes' }],
    footer: {
      message:
        'Released under the <a href="https://github.com/kagenokeiyou/ros-notes/blob/main/LICENSE">MIT License</a>',
      copyright: 'Copyright © 2026-present <a href="https://github.com/kagenokeiyou">Keiyou</a>',
    },
    outline: {
      level: 'deep',
    },

    sidebar: [
      {
        text: '开始',
        collapsed: true,
        items: [
          { text: '什么是 ROS', link: '/01-start/01-what-is-ros.md' },
          { text: '安装 ROS2', link: '/01-start/02-install-ros2.md' },
          { text: '运行 turtlesim', link: '/01-start/03-run-turtlesim.md' },
        ],
      },
      {
        text: '节点',
        collapsed: true,
        items: [
          { text: '编写节点', link: '/02-node/01-code-node.md' },
          { text: '使用功能包', link: '/02-node/02-use-package.md' },
          { text: '使用工作空间', link: '/02-node/03-use-workplace.md' },
        ],
      },
    ],
  },
})
