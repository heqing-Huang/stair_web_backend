# https://blog.csdn.net/y472360651/article/details/78538188
# 并行工作进程数
workers = 2
# 指定每个工作者的线程数
threads = 8
# 监听内网端口8000
bind = "0.0.0.0:80"
# 设置最大并发量
worker_connections = 8
# 设置日志记录水平
loglevel = "info"
# uvicorn 方式
worker_class = "uvicorn.workers.UvicornWorker"
# 决定重启
max_requests = 2000
# 抖动
max_requests_jitter = 1
# 超时杀掉
timeout = 30
keepalive = 8
# 尝试
pidfile = "/tmp/server.pid"
