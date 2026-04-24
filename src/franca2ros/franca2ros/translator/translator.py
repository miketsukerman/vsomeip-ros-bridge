from pyfranca import Processor, LexerException, ParserException, ProcessorException
from jinja2 import FileSystemLoader, Environment

import os
dir_path = os.path.dirname(os.path.realpath(__file__))


class FIDLTranslator:

    def __init__(self, fidl, include):
        self.fidl = fidl
        self.include = include

        templateLoader = FileSystemLoader(searchpath=f"{dir_path}/templates/")
        templateEnv = Environment(loader=templateLoader)

        self.message_t = templateEnv.get_template("message.msg.j2")
        self.service_t = templateEnv.get_template("service.srv.j2")

    def parse(self):
        processor = Processor()
        types = []
        services = []

        try:
            print(f"importing file {self.fidl}")
            processor.import_file(self.fidl)

            for package in processor.packages.values():
                if package.typecollections:
                    print("\tType collections:")
                    for typecollection in package.typecollections.values():
                        if typecollection.structs:
                            print("\t\tStructs:")
                            for item in typecollection.structs.values():
                                print("\t\t- {}".format(item.name))
                                types.append(item)

                if package.interfaces:
                    print("\tInterfaces:")
                    for interface in package.interfaces.values():
                        if interface.methods:
                            print("\t\tMethods:")
                            for method in interface.methods.values():
                                print("\t\t- {}".format(method.name))
                                services.append(method)

        except (LexerException, ParserException, ProcessorException) as e:
            print(f"ERROR: {e}")

        return types, services

    def fidlTypes2RosTypes(self, name):
        types_map = {
            "Float": "float32",
            "Double": "float64",
            "Int8": "int8",
            "Int16": "int16",
            "Int32": "int32",
            "Int64": "int64",
            "UInt8": "uint8",
            "UInt16": "uint16",
            "UInt32": "uint32",
            "UInt64": "uint64",
            "Boolean": "bool",
            "String": "string",
            "ByteBuffer": "uint8[]",
        }
        return types_map[name] if name in types_map else name

    def save(self, output):
        types, services = self.parse()

        for type_ in types:
            output_file = os.path.join(output, f"{type_.name}.msg")
            with open(output_file, "w+") as f:
                print(f"saving to {output_file}")
                print(type_.fields)
                out = self.message_t.render(fields=type_.fields, rostype=self.fidlTypes2RosTypes)
                f.write(out)

        for service in services:
            output_file = os.path.join(output, f"{service.name}.srv")
            with open(output_file, "w+") as f:
                print(f"saving to {output_file}")
                out = self.service_t.render(
                    in_args=service.in_args if service.in_args else {},
                    out_args=service.out_args if service.out_args else {},
                    rostype=self.fidlTypes2RosTypes
                )
                f.write(out)
