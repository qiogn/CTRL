# Claude Code 工作流操作指南：Agent + Skills + 版本管理

## 1. 创建 Agent
**操作步骤（Claude Code 可执行）：**
```bash
claude-code agent create --name DataProcessor --type task
claude-code agent create --name TaskCoordinator --type coordinator
```
> Claude Code 会生成 `agents/DataProcessor.py` 和 `agents/TaskCoordinator.py` 文件。

## 2. 注册 Agent 到工作流
```python
from agents.DataProcessor import DataProcessor
from agents.TaskCoordinator import TaskCoordinator

workflow.register(DataProcessor())
workflow.register(TaskCoordinator())
```
**验证：**
```bash
claude-code workflow test
```

## 3. 创建 Skills
在 `skills/` 目录创建 Skill 文件：
```python
# skills/DataAnalysisSkill.py
class DataAnalysisSkill:
    def execute(self, data):
        return processed_data

# skills/NotificationSkill.py
class NotificationSkill:
    def execute(self, message):
        send_notification(message)
```

## 4. 绑定 Skills 到 Agent
```python
from skills.DataAnalysisSkill import DataAnalysisSkill
from skills/NotificationSkill import NotificationSkill

# DataProcessor 绑定 DataAnalysisSkill
self.add_skill(DataAnalysisSkill())

# TaskCoordinator 绑定 NotificationSkill
self.add_skill(NotificationSkill())
```
**验证：**
```bash
claude-code agent list-skills --agent DataProcessor
claude-code agent list-skills --agent TaskCoordinator
```

## 5. 测试工作流交互
```python
data = load_data()
result = workflow.execute_agent("DataProcessor", "DataAnalysisSkill", data)
workflow.execute_agent("TaskCoordinator", "NotificationSkill", f"处理完成: {result}")
```
```bash
claude-code workflow run
```

## 6. 集成代码版本管理

### 6.1 初始化版本库
```bash
git init
git add .
git commit -m "初始化 Claude Code 工作流"
```

### 6.2 每次更新存档
1. 修改 Agent、Skill 或工作流代码。
2. 提交更新：
```bash
git add .
git commit -m "更新 Agent 或 Skill: 说明本次修改内容"
```
3. 可选：创建分支管理不同功能：
```bash
git checkout -b feature/new_skill
git commit -m "添加新 Skill"
git checkout main
git merge feature/new_skill
```

### 6.3 查看历史记录
```bash
git log --oneline
```

## 7. 最佳实践

- **命名规范**：Agent 和 Skill 命名保持一致，便于管理。
- **日志与监控**：使用 Claude Code CLI 查看 Agent 执行日志。
- **版本控制**：每次改动都提交 commit，并写清楚修改内容。
- **分支管理**：不同功能或实验使用独立分支，避免主分支混乱。

## 8. 总结

通过此指南，你可以：
1. 在 Claude Code 工作流中添加两个 Agent。
2. 给每个 Agent 配置独立 Skill。
3. 使用 Git 进行代码版本管理，每次更新都有存档。
4. 测试和监控整个工作流。

