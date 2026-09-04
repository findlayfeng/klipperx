# 测试

测试位于 `src/core/klippy/msg/parser.rs` 的 `#[cfg(test)]` 模块，使用 `TestInterface` 模拟底层 IO。覆盖：

- 发送：位置参数、命名参数、混合、类型转换、各错误路径
- 绑定：命令绑定、绑定后禁止发送、未知命令
- 回调：消息路由、回调队列 drain
- send_and_wait：正常返回、超时、inbox 未启动、发送失败后清理 waiter
- 并发：同消息名多个 waiter 的精确移除

## 运行测试

```bash
cargo test --lib msg
```

---

- [← 开发手册首页](README.md)
- [内部架构 ←](architecture.md)
