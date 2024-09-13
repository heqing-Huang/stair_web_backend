# 搭建开发环境

## 一. python 解释器选择

**conda python >=3.8.16 && < 3.9**

- 因为我们需要在内部使用occ扩展（stair_dxf),故**conda**是必须的.
- 使用了django中的jsonfield,更方便的使用方式是让**python>=3.8.16**
- 对dataclass做了强制类型的扩充,这导致在高版本的python上无法正常运行——后续的数据类,
  计划使用attrs来实现,他在功能上,是dataclass的超集.

## 二. 安装依赖

**注意**:

pip 会通过git ssh 远程安装,请确保配置本地git ssh,已经git账号具有对所依赖仓库的访问权限.

### PIP

- 本地初次安装

```shell
pip install -r requirements.txt
```

- 本地更新安装

```shell
pip install --upgrade -r requirements.txt
```

**注意**:

从库的设计来说,会有兼容性可言。即，选择老版本应该处于可用状态。

但目前下层依赖的多个工具库，无法维持向前兼容——短期内需求变化大——故在依赖的申明上，并不严格。

可能实际安装后，程序无法正常使用——此时，需主动提ISSUE 以完善对依赖的申明。

### CONDA

```shell
conda install -y -c conda-forge pythonocc-core
```

目前，occ 在图纸正向生成部分是刚需的。

但是setuptools 无法申明conda 的依赖，所以需要手动安装。

**注意**:

此前`ifcopenshell`需要通过conda 进行安装.但目前pypi 上已经存在了ifcopenshell的托管.
故将其转移进入了`requirements.txt`.

## 三. 数据库配置相关

### 同步数据库表结构

```shell
python manage.py migrate
```

### 创建超级管理员

```shell
python manage.py createsuperuser
```

**注意**:

该账号将用于登录django网站,具有最高权限

## 四. 启动

需要首先配置好后台的多个服务,web端运行才可以正常启动

### 后台 服务

#### 队列服务

```shell
docker run -d --name rabbit-stair-web-backend -p 15672:15672 -p 5672:5672 -e RABBITMQ_DEFAULT_USER=test -e RABBITMQ_DEFAULT_PASS=test_pwd rabbitmq:3-management
```

**注意**:

- 以上,仅为基于docker,快速运行rabbitmq 的示例.各自开发环境可选择其他安装rabbitmq的方式
- rabbitmq的账号密码在程序中固定,如果通过安装rabbitmq的方式运行队列服务，则需要创建与之匹配的账号密码
- rabbitmq 用于解耦,将耗时的计算任务,分发给worker 服务

#### Worker 服务

windows

```shell
celery -A stairs worker -l info -P solo
```

or linux

```shell
celery -A stairs worker -l info
```

**注意**:

- windows 下的运行方式仅能用于demo.`-P solo` 性能十分有限,且高版本`celery`
  对windows支持十分有限。
- worker 服务可以和web 服务运行在不同电脑。解耦由rabbitmq完成。

### web 服务

```shell
python manage.py runserver 127.0.0.1:8000
```

然后[点击访问](http://127.0.0.1:8000/admin)

**注意**:

该方式,仅能用于django的开发调试.生产环境的部署,请参阅django 部署文档.
