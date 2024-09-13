FROM datajoint/miniconda3:22.11.1-py3.9-debian-72c2c4b
LABEL authors="lanhao945"
ENV TZ=Asiz/Shanghai
WORKDIR /project
USER root
RUN sed -i 's#http://deb.debian.org#https://mirrors.ustc.edu.cn#g' /etc/apt/sources.list
RUN sed -i 's|security.debian.org/debian-security|mirrors.ustc.edu.cn/debian-security|g' /etc/apt/sources.list
RUN apt-get update
RUN apt-get install -y libpq-dev
RUN apt-get install -y python3-dev
RUN apt-get install -y gcc
RUN conda install -y -c conda-forge pythonocc-core
COPY ./requirements.txt /project/requirements.txt
RUN pip install --no-cache-dir -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
COPY . /project
EXPOSE 80
CMD gunicorn -c gunicorn.py stairs.asgi:application
