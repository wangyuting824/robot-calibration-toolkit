"""Optional Python socket guard used only for offline verification runs."""
import socket


def install():
    if getattr(socket, "_robotcal_guard", False):
        return
    socket._robotcal_guard = True
    original_connect = socket.socket.connect
    original_connect_ex = socket.socket.connect_ex

    def connect(sock, address):
        if sock.family in (socket.AF_INET, socket.AF_INET6):
            raise RuntimeError("ROBOTCAL_DISABLE_NETWORK: network connection forbidden in this verification run")
        return original_connect(sock, address)

    def connect_ex(sock, address):
        if sock.family in (socket.AF_INET, socket.AF_INET6):
            raise RuntimeError("ROBOTCAL_DISABLE_NETWORK: network connection forbidden in this verification run")
        return original_connect_ex(sock, address)

    socket.socket.connect = connect
    socket.socket.connect_ex = connect_ex
