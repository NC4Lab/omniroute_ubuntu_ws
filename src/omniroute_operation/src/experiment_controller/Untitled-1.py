

class Person:
    def __init__(self, name, age):
        self.name = name
        self.age = age+5

    def say_hello(self):
        print(f'Hello, my name is {self.name} and I am {self.age} years old')


if __name__ == '__main__':
    elena = Person('Elena', 22)
    elena.say_hello()

    john = Person('John', 30)
    john.say_hello()