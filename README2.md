# stair_web_backend

楼梯深化设计的后端实现

## 开发环境准备

- python 环境

python 3.8

```shell
pip install -r requirements.txt
```
```shell
pip install stair_ifc@git+ssh://git@github.com/IBLofCQU/stair_ifc.git
```
**注意**:pip 会通过git ssh 远程安装,请确保配置本地git ssh,已经git账号具有对所依赖仓库的访问权限.

- 同步数据库表结构

```shell
python manage.py migrate
```

- 创建超级管理员

```shell
python manage.py createsuperuser
```

**注意**:该账号用于登录django网站,具有最高权限

- 运行web 服务

```shell
python manage.py runserver 127.0.0.1:8000
```
然后[点击访问](http://127.0.0.1:8000/admin)
