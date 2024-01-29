from queue import PriorityQueue
from dataclasses import dataclass, field

open = PriorityQueue()


@dataclass(order=True)
class Item:
    priority: int
    distance_tie: int
    name: str=field(compare=False)


open.put_nowait(Item(priority=0, distance_tie=0.0, name="a"))
open.put_nowait(Item(priority=2, distance_tie=1.0, name="b"))
open.put_nowait(Item(priority=1, distance_tie=0.9, name="c"))
open.put_nowait(Item(priority=2, distance_tie=1.1, name="d"))
open.put_nowait(Item(priority=1, distance_tie=0.7, name="e"))

while not open.empty():
    current = open.get_nowait()
    print(current.name)