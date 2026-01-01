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

    sidebar: [
      {
        text: '开始',
        collapsed: true,
        items: [{ text: '什么是 ROS', link: '/01-start/01-what-is-ros.md' }],
      },
    ],
  },
})
