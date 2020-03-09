import Pyro4
import threading
import time

@Pyro4.expose
@Pyro4.behavior(instance_mode="single")
class Interface(object):
    def report_result(self, result):
        print('result: ', result['id'], result['distance'])
class Server():
    def enable(self):
        self.daemon = Pyro4.Daemon(port=53546)
        self.daemon.register(Interface, "interface")
        self.thread = threading.Thread(target=self.daemonLoop)
        self.thread.start()
        print("Started sim coordinator server")

    def disable(self):
        print("Called for daemon shutdown")
        self.daemon.shutdown()

    def daemonLoop(self):
        self.daemon.requestLoop()
        print("Daemon has shut down no prob")

if __name__ == '__main__':
    s = Server()
    s.enable()
    # s.disable()
