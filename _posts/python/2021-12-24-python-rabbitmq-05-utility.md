---
layout: post
title: Python에서 RabbitMq 사용하기 (5) - RabbitMq Utlity
category: Python
tag: [Python, RabbitMQ]
---

# RabbitMq Utlity

## rabbitmq_util.py

<pre class="prettyprint">
from http.client import HTTPConnection
import json
from urllib.parse import quote
from base64 import b64encode


class RabbitMqUtil:
    __OVERVIEW_URL = "/api/overview"
    __VHOSTS_URL = "/api/vhosts"
    __USERS_URL = "/api/users"
    __EXCHANGES_URL = "/api/exchanges"
    __QUEUES_URL = "/api/queues"
    __BINDINGS_URL = "/api/bindings"

    def __init__(self, host="localhost", port=15672, user="guest", passwd="guest"):
        self.__host = host
        self.__port = port
        encoded_auth = b64encode(f"{user}:{passwd}".encode()).decode()
        self.__header = {"Authorization": f"Basic {encoded_auth}",
                         "content-type": "application/json"}

    def __request(self, method, url, body=None):
        connection = HTTPConnection(self.__host, self.__port, timeout=5)
        connection.request(method=method,
                           url=url,
                           headers=self.__header,
                           body=body)
        response = connection.getresponse()
        code = response.getcode()
        content = response.read()
        response.close()
        connection.close()

        return code, content

    def get_overview(self):
        return self.__request("GET", RabbitMqUtil.__OVERVIEW_URL)

    def get_users(self):
        result = []
        code, content = self.__request("GET", RabbitMqUtil.__USERS_URL)
        users = json.loads(content)
        for item in users:
            result.append(item["name"])

        return result

    def get_vhosts(self):
        result = []
        code, content = self.__request("GET", RabbitMqUtil.__VHOSTS_URL)
        vhosts = json.loads(content)
        for item in vhosts:
            result.append(item["name"])

        return result

    def create_vhosts(self, name):
        name = quote(name, "")
        code, content = self.__request("PUT", f"{RabbitMqUtil.__VHOSTS_URL}/{name}")

        return (code == 201) or (code == 204)

    def delete_vhosts(self, name):
        name = quote(name, "")
        code, content = self.__request("DELETE", f"{RabbitMqUtil.__VHOSTS_URL}/{name}")

        return (code == 201) or (code == 204)

    def get_exchanges(self, vhost):
        result = []
        vhost = quote(vhost, "")
        code, content = self.__request("GET", f"{RabbitMqUtil.__EXCHANGES_URL}/{vhost}")
        exchanges = json.loads(content)
        for item in exchanges:
            result.append(item["name"])

        return result

    def create_exchange(self, vhost, exchange,
                        exchange_type="fanout",
                        auto_delete=False,
                        durable=True,
                        internal=False,
                        arguments=None):
        vhost = quote(vhost, '')
        exchange = quote(exchange, '')
        base_body = {"type": exchange_type,
                     "auto_delete": auto_delete,
                     "durable": durable,
                     "internal": internal,
                     "arguments": arguments or list()}
        body = json.dumps(base_body)
        code, content = self.__request("PUT", f"{RabbitMqUtil.__EXCHANGES_URL}/{vhost}/{exchange}", body)

        return (code == 201) or (code == 204)

    def delete_exchange(self, vhost, exchange):
        vhost = quote(vhost, '')
        exchange = quote(exchange, '')
        code, content = self.__request("DELETE", f"{RabbitMqUtil.__EXCHANGES_URL}/{vhost}/{exchange}")

        return (code == 201) or (code == 204)

    def get_queues(self, vhost):
        result = []
        vhost = quote(vhost, "")
        code, content = self.__request("GET", f"{RabbitMqUtil.__QUEUES_URL}/{vhost}")
        exchanges = json.loads(content)
        for item in exchanges:
            result.append(item["name"])

        return result

    def create_queue(self, vhost, queue, **kwargs):
        vhost = quote(vhost, '')
        queue = quote(queue, '')
        body = json.dumps(kwargs)
        code, content = self.__request("PUT", f"{RabbitMqUtil.__QUEUES_URL}/{vhost}/{queue}", body)

        return (code == 201) or (code == 204)

    def delete_queue(self, vhost, queue):
        vhost = quote(vhost, '')
        queue = quote(queue, '')
        code, content = self.__request("DELETE", f"{RabbitMqUtil.__QUEUES_URL}/{vhost}/{queue}")

        return (code == 201) or (code == 204)

    def purge_queue(self, vhost, queue):
        vhost = quote(vhost, '')
        queue = quote(queue, '')
        code, content = self.__request("DELETE", f"{RabbitMqUtil.__QUEUES_URL}/{vhost}/{queue}/contents")

        return (code == 201) or (code == 204)

    def get_bindings(self, vhost):
        vhost = quote(vhost, '')
        code, content = self.__request("GET", f"{RabbitMqUtil.__BINDINGS_URL}/{vhost}")

        return content

    def get_queue_bindings(self, vhost, queue):
        vhost = quote(vhost, '')
        queue = quote(queue, '')
        code, content = self.__request("GET", f"{RabbitMqUtil.__QUEUES_URL}/{vhost}/{queue}/bindings")

        return content

    def create_binding(self, vhost, exchange, queue, routing_key="", args=None):
        vhost = quote(vhost, '')
        exchange = quote(exchange, '')
        queue = quote(queue, '')
        body = json.dumps({"routing_key": routing_key, "arguments": args or {}})
        code, content = self.__request("POST",
                                       f"{RabbitMqUtil.__BINDINGS_URL}/{vhost}/e/{exchange}/q/{queue}",
                                       body)

        return (code == 201) or (code == 204)

    def delete_binding(self, vhost, exchange, queue, routing_key):
        vhost = quote(vhost, '')
        exchange = quote(exchange, '')
        queue = quote(queue, '')
        body = ''
        code, content = self.__request("DELETE",
                                       f"{RabbitMqUtil.__BINDINGS_URL}/{vhost}/e/{exchange}/q/{queue}/{routing_key}",
                                       body)

        return (code == 201) or (code == 204)
</pre>

## main.py

테스트용 코드로 별도로 필요하진 않습니다.

<pre class="prettyprint">
from pyrabbit.rabbitmq_util import RabbitMqUtil

rabbit = RabbitMqUtil("localhost")

VHOST_NAME = "snowdeer_vhost"
EXCHANGE_NAME = "snowdeer_exchange"
QUEUE_NAME = "snowdeer_queue"


def main():
    print("&lt;OVERVIEW&gt;")
    overview = rabbit.get_overview()
    print(f"overview: {overview}")

    test_users()
    test_vhosts(VHOST_NAME)
    test_exchanges(VHOST_NAME, EXCHANGE_NAME)
    test_queues(VHOST_NAME, QUEUE_NAME)
    test_bindings(VHOST_NAME, EXCHANGE_NAME, QUEUE_NAME)
    # delete_resources(VHOST_NAME, EXCHANGE_NAME, QUEUE_NAME)


def test_users():
    print("\n&lt;TEST USERS&gt;")

    users = rabbit.get_users()
    print(f"users: {users}")


def test_vhosts(vhost_name):
    print("\n&lt;TEST VHOSTS&gt;")

    vhosts = rabbit.get_vhosts()
    print(f"vhosts: {vhosts}")

    ret = rabbit.create_vhosts(vhost_name)
    print(f"create vhost: {ret}")

    vhosts = rabbit.get_vhosts()
    print(f"vhosts: {vhosts}")


def test_exchanges(vhost, exchange):
    print("\n&lt;TEST EXCHANGES&gt;")

    exchanges = rabbit.get_exchanges(vhost)
    print(f"exchanges: {exchanges}")

    ret = rabbit.create_exchange(vhost, exchange)
    print(f"create exchanges: {ret}")

    exchanges = rabbit.get_exchanges(vhost)
    print(f"exchanges: {exchanges}")


def test_queues(vhost, queue):
    print("\n&lt;TEST QUEUES&gt;")

    queues = rabbit.get_queues(vhost)
    print(f"queues: {queues}")

    ret = rabbit.create_queue(vhost, queue)
    print(f"create queue: {ret}")

    queues = rabbit.get_queues(vhost)
    print(f"queues: {queues}")

    ret = rabbit.purge_queue(vhost, queue)
    print(f"purge queue: {ret}")


def test_bindings(vhost, exchange, queue):
    print("\n&lt;TEST BINDINGS&gt;")

    bindings = rabbit.get_bindings(vhost)
    print(f"queues: {bindings}")

    bindings = rabbit.get_queue_bindings(vhost, queue)
    print(f"queues: {bindings}")

    ret = rabbit.create_binding(vhost, exchange, queue, routing_key="hello")
    print(f"create binding: {ret}")

    bindings = rabbit.get_bindings(vhost)
    print(f"queues: {bindings}")

    bindings = rabbit.get_queue_bindings(vhost, queue)
    print(f"queues: {bindings}")


def delete_resources(vhost, exchange, queue):
    ret = rabbit.delete_queue(vhost, queue)
    print(f"delete queue: {ret}")

    queues = rabbit.get_queues(vhost)
    print(f"queues: {queues}")

    ret = rabbit.delete_exchange(vhost, exchange)
    print(f"delete exchanges: {ret}")

    exchanges = rabbit.get_exchanges(vhost)
    print(f"exchanges: {exchanges}")

    ret = rabbit.delete_vhosts(VHOST_NAME)
    print(f"delete vhost: {ret}")

    vhosts = rabbit.get_vhosts()
    print(f"vhosts: {vhosts}")


def test_apis():
    rabbit.create_vhosts("test_vhost")

    rabbit.create_exchange("test_vhost", "ex1")
    rabbit.create_exchange("test_vhost", "ex2")

    rabbit.create_queue("test_vhost", "q1")
    rabbit.create_queue("test_vhost", "q2")
    rabbit.create_queue("test_vhost", "q3")

    rabbit.create_binding("test_vhost", "ex1", "q1")
    rabbit.create_binding("test_vhost", "ex2", "q2", routing_key="hello")
    rabbit.create_binding("test_vhost", "ex2", "q3", routing_key="snowdeer_binding")


if __name__ == '__main__':
    main()
    # test_apis()
</pre>