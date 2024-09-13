### v0.0.8

- 后台补充了对IFC 调用的生成

开发环境的配置文档改变,主要包括:

- `ifcopenshll` 支持pip 直接安装,不再需要conda 环境
- `sqlite` `json filed` 的支持,在低版本python下无,提高python版本以避免问题
- celery worker eventlet
  线程池会出现无法写入文件的错误（依旧对windows支持很差）,后续可能会调整开发环境和运行环境（linux）

### v0.0.7

- 对下层stair-detailed 1.0.0 版本的参数新增做了适配

### v0.0.6

- 取消深化设计返回中RailParameter 相关参数得not null 限制

### v0.0.5

- 移除显式 git+ssh 安装

### v0.0.4

- 完成滴水槽在软件层的参数一致性校验——model save 前

### v0.0.3

- 修正表结构中的enum value 的错误使用一处

### v0.0.2

- 在结构计算参数录入界面新增项目相关参数

### v0.0.1

- 适配下层深化设计和结构设计参数的变动
