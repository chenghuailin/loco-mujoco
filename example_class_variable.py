#!/usr/bin/env python3

class BaseClass:
    # 类变量 - 被所有实例和子类共享
    shared_registry = {}
    
    @classmethod
    def register(cls):
        cls_name = cls.__name__
        BaseClass.shared_registry[cls_name] = cls
        print(f"注册了 {cls_name}")
    
    @classmethod
    def list_registered(cls):
        return list(BaseClass.shared_registry.keys())

class ChildA(BaseClass):
    pass

class ChildB(BaseClass):
    pass

class ChildC(BaseClass):
    pass

# 演示类变量的共享性
if __name__ == "__main__":
    print("=== 演示类变量共享 ===")
    
    # 注册不同的子类
    ChildA.register()
    ChildB.register()
    ChildC.register()
    
    print(f"\n所有注册的类: {BaseClass.list_registered()}")
    
    # 验证所有类都能访问同一个注册表
    print(f"通过BaseClass访问: {BaseClass.shared_registry}")
    print(f"通过ChildA访问: {ChildA.shared_registry}")
    print(f"通过ChildB访问: {ChildB.shared_registry}")
    
    # 验证它们指向同一个对象
    print(f"\n是否是同一个对象: {BaseClass.shared_registry is ChildA.shared_registry}")
    print(f"内存地址相同: {id(BaseClass.shared_registry) == id(ChildA.shared_registry)}")
