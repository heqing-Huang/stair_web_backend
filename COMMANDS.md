# 存放开发期间常用的命令

- 同步表结构

**需按序执行**

orm 生成 sql 命令

```shell
python manage.py makemigrations
```

sql 命令同步到 数据库文件

```shell
python manage.py migrate
```

- 单元测试

```shell
python manage.py test
```

- 重新安装

```shell
pip install -r requirements.txt
```

- 静态文件

```shell
python manage.py collectstatic
```
