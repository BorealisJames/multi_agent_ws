class BorealisNodeResource:

    def __init__(self, node_name):

        self.node_name = node_name
        self.node_socket = None
        self.uri_address = None
        self.time_stamp =[]
        self.cpu_usage = []
        self.network_usage = []
