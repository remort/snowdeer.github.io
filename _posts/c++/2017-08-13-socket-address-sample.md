---
layout: post
title: SocketAddress 및 SocketAddressFactory 예제
category: C++
tag: [C++, 네트워크]
---
# SocketAddress.h

<pre class="prettyprint">
#ifndef SNOWSOCKET_SOCKETADDRESS_H
#define SNOWSOCKET_SOCKETADDRESS_H

#include &lt;sys/socket.h&gt;
#include &lt;netinet/in.h&gt;
#include &lt;arpa/inet.h&gt;
#include &lt;cstring&gt;
#include &lt;memory&gt;
#include &lt;w32api/inaddr.h&gt;

using namespace std;

class SocketAddress {
 public:
  SocketAddress(uint32_t address, uint16_t port) {
    getSockAddrIn()->sin_family = AF_INET;
    getSockAddrIn()->sin_addr.s_addr = htonl(address);
    getSockAddrIn()->sin_port = htons(port);
  }

  SocketAddress(string address, uint16_t port) {
    getSockAddrIn()->sin_family = AF_INET;
    getSockAddrIn()->sin_addr.s_addr = inet_addr(address.c_str());
    getSockAddrIn()->sin_port = htons(port);
  }

  SocketAddress(const sockaddr &sockAddr) {
    memcpy(&mSockAddr, &sockAddr, sizeof(sockaddr));
  }

  size_t size() const {
    return sizeof(sockaddr);
  }

 private:
  sockaddr mSockAddr;
  sockaddr_in *getSockAddrIn() {
    return reinterpret_cast&lt;sockaddr_in *&gt; (&mSockAddr);
  }
};

using SocketAddressPtr = shared_ptr&lt;SocketAddress&gt;;

#endif //SNOWSOCKET_SOCKETADDRESS_H
</pre>

<br>

# SocketAddressFactory.h

<pre class="prettyprint">
#ifndef SNOWSOCKET_SOCKETADDRESSFACTORY_H
#define SNOWSOCKET_SOCKETADDRESSFACTORY_H

#include &lt;cstdio&gt;
#include &lt;netdb.h&gt;
#include "SocketAddress.h"

using namespace std;

class SocketAddressFactory {
 public:
  static SocketAddressPtr createIPv4SocketAddress(const string &address) {
    string host, service;

    auto pos = address.find_last_of(':');
    if (pos != string::npos) {
      host = address.substr(0, pos);
      service = address.substr(pos + 1);
    } else {
      host = address;
      service = "0";
    }

    addrinfo hint;
    memset(&hint, 0, sizeof(hint));
    hint.ai_family = AF_INET;

    addrinfo *_result = nullptr;
    int error = getaddrinfo(host.c_str(), service.c_str(), &hint, &_result);
    addrinfo *result = _result;

    if (error != 0 && _result != nullptr) {
      freeaddrinfo(result);

      return nullptr;
    }

    while (!_result->ai_addr && _result->ai_next) {
      _result = _result->ai_next;
    }

    if (!_result->ai_addr) {
      freeaddrinfo(result);

      return nullptr;
    }

    auto toRet = make_shared&lt;SocketAddress&gt;(*_result->ai_addr);

    freeaddrinfo(result);

    return toRet;
  }
};

#endif //SNOWSOCKET_SOCKETADDRESSFACTORY_H
</pre>