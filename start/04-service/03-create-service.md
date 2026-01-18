# 创建服务

```python
self.create_service(
    srv_type,
    srv_name,
    callback
)
```

创建一个 Service 服务端

- `srv_type` Service 类型（如 AddTwoInts）
- `srv_name` Service 名称（如 'add_two_ints'）
- `callback` 请求处理函数
