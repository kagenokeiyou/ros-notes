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
    nav: [
      { text: '开始', link: '/start/01-start/01-what-is-ros.md' },
      { text: '概念', link: '/concepts/01-basic/01-node.md' },
      { text: 'API', link: '/api/01-start.md' },
    ],

    sidebar: {
      '/start/': [
        {
          text: '开始',
          collapsed: true,
          items: [
            { text: '什么是 ROS', link: '/start/01-start/01-what-is-ros.md' },
            { text: '安装 ROS2', link: '/start/01-start/02-install-ros2.md' },
            { text: '配置环境', link: '/start/01-start/03-config-env.md' },
            { text: '运行 turtlesim', link: '/start/01-start/04-run-turtlesim.md' },
          ],
        },
        {
          text: '节点',
          collapsed: true,
          items: [
            { text: '编写节点', link: '/start/02-node/01-code-node.md' },
            { text: '使用功能包', link: '/start/02-node/02-use-package.md' },
            { text: '使用工作空间', link: '/start/02-node/03-use-workplace.md' },
          ],
        },
        {
          text: '话题',
          collapsed: true,
          items: [
            { text: '话题通信介绍', link: '/start/03-topic/01-introduction.md' },
            { text: '发布小说', link: '/start/03-topic/02-pub-novel.md' },
            { text: '订阅小说', link: '/start/03-topic/03-sub-novel.md' },
          ],
        },
        {
          text: '服务',
          collapsed: true,
          items: [
            { text: '服务通信介绍', link: '/start/04-service/01-introduction.md' },
            { text: '参数通信', link: '/start/04-service/02-parameter.md' },
            { text: 'launch', link: '/start/04-service/04-launch.md' },
          ],
        },
        {
          text: '工具',
          collapsed: true,
          items: [
            { text: 'tf', link: '/start/05-tools/01-tf.md' },
            { text: 'rqt ', link: '/start/05-tools/02-rqt.md' },
            { text: 'rviz', link: '/start/05-tools/03-rviz.md' },
            { text: 'bag', link: '/start/05-tools/04-bag.md' },
          ],
        },
        {
          text: '模拟',
          collapsed: true,
          items: [
            { text: 'URDF', link: '/start/06-sim/01-urdf.md' },
            { text: 'Xacro', link: '/start/06-sim/02-xacro.md' },
            { text: 'Gazebo', link: '/start/06-sim/03-gazebo.md' },
          ],
        },
      ],
      '/concepts/': [
        {
          text: '基本概念',
          collapsed: false,
          items: [
            { text: '节点（Node）', link: '/concepts/01-basic/01-node.md' },
            { text: '发现（Discovery）', link: '/concepts/01-basic/02-discovery.md' },
            { text: '接口（Interface）', link: '/concepts/01-basic/03-interface.md' },
            { text: '消息（Message）', link: '/concepts/01-basic/04-message.md' },
            { text: '主题（Topic）', link: '/concepts/01-basic/05-topic.md' },
            { text: '服务（Service）', link: '/concepts/01-basic/06-service.md' },
            { text: '动作（Action）', link: '/concepts/01-basic/07-action.md' },
            { text: '参数（Parameter）', link: '/concepts/01-basic/08-parameter.md' },
            {
              text: '使用命令行工具进行内省',
              link: '/concepts/01-basic/09-command-introspection.md',
            },
            { text: '启动（launch）', link: '/concepts/01-basic/10-launch.md' },
            { text: '客户端库', link: '/concepts/01-basic/11-client-libraries.md' },
          ],
        },
        {
          text: '进阶概念',
          collapsed: false,
          items: [{ text: 'ROS_DOMAIN_ID', link: '/concepts/02-intermediate/01-domain-id.md' }],
        },
        {
          text: '高级概念',
          collapsed: false,
        },
      ],
      '/api/': [
        {
          items: [
            { text: '开始', link: '/api/01-start.md' },
            { text: '基础', link: '/api/02-basic.md' },
          ],
        },
      ],
    },
  },
})
