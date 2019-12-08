# Rubik 2x2
## Môi trường cài đặt :
1. Ngôn ngữ : **Python 3**
2. Thư viện ngoài sử dụng : **Pygame**
```
pip install pygame
```

## Chạy chương trình 
_ Các thông số khi chạy chương trình : 
``` 
-h, --help  Hiện ra thông tin thêm khi chạy chương trình 
--m False   Chạy bằng tay (Sử dụng GUI) hoặc chạy giải thuật trước 
--n 2       Kích thước khối rubik 
--s 5       Số lần trộn rubik 
--a ida*    Thuật toán sử dụng : bfs, bbfs, a*, ida*
--h m    Hàm heuristic sử dụng : s-simpleHeuristic, h-hammingDistance, m-manhattanDistance 
```

_ Chạy với các thông số mặc định 
1. n = 2
2. s = 5
3. a = ida*
4. h = m 
```
python3 main.py 
```

## Cấu trúc thư mục 
```
├── main.py                 # Hàm main để nhận các thông số chạy chương trình
├── main_gui.py             # Xây dựng đồ họa cho chương trình
├── AIs.py                  # Chứa các thuật toán để giải rubik
├── Heuristic.py            # Chứa các hàm Heuristic
├── Cube.py                 # Tạo model khối rubik
└── ManhattanCube.py        # Tạo model Mahattan Rubik phục vụ cho hàm manhattanDistance
```

## main.py 
_ Import : 
```python
import time, argparse
from main_gui import *
from AIs import *
from Heuristic import * 
```
_ Hàm chạy toàn bộ chương trình 

_ Cho phép người dùng tự giải rubik trên GUI hoặc lựa chọn các thuật giải và hàm heuristic để giải rubik 
```python
# Khởi động GUI
# n = Kích thước rubik
# scramble_length = Số lần trộn
gui(n, scramble_length)


# Chạy với giải thuật và hàm heuristic lựa chọn
# type = giải thuật
# n = kích thước
# scramble_length = số lần trộn
# heuristic = hàm heuristic sử dụng cho giaỉ thuật
ai(type, n, scramble_length, heuristic)
```

## main_gui.py 
_ Import : 
```python
import sys, math, pygame, time
from operator import itemgetter
from Cube import *
from copy import copy, deepcopy 
```
_ File tạo ra toàn bộ GUI để hiển thị. 

_ GUI bao gồm cả 2D và 3D của rubik và cho phép xem đường đi được giải bởi AI 

_ **class GUI** : Để xuất hình ảnh cả 2D và 3D ra màn hình cũng như nhận sự kiện để chuyển đổi giữa 2 góc nhìn 

_ **class Point3D** : Tạo ra hình ảnh 3D của rubik. Tham khảo từ http://codeNtronix.com và được phát triển bởi Leonel Machava 

_ **class ThreeD_Cube** : Di chuyển rubik 3D 

## AIs.py 
_ Import : 
```python
import heapq, time
from Cube import *
from Heuristic import *
```
_ Bao gồm các giải thuật để giải rubik 

_ Đường đi được tạo ra dưới dạng tuples với mỗi phần tử gồm bước đi và state được tạo ra bởi bước đi đó  

_ **class BFS** : 
```python
__init__(cube)
solve(timeout=float('inf'))
find_path(self, seen, goal_state)
```
_ **class BBFS** : 
```python
__init__(cube)
solve(timeout=float('inf'))
find_path(self, seen, goal_state)
```

_ **class A_Star** :
```python
# Mặc định là thuật toán manhattan
__init__(cube, heuristic=Heuristic.manhattanDistance)
solve(timeout=float('inf'))

#Tìm lại đường đi từ state ban đầu để state đích
find_path(start_state, end_state)
```

_ **class IDA_Star** : 
```python
# Mặc định là thuật toán manhattan
__init__(cube, heuristic=Heuristic.manhattanDistance)
solve(timeout=float('inf'))

# Tìm kiếm sâu dần 
# path = đường đi hiện tại 
# g = độ sâu hiện tại 
# bound = hàm cận trên
search(path, g, bound, times=(float('inf'),0))
```

_ **class State** : 
```python
__init__(current_state, parent_state, fValue, depth, move)
# So sánh xem 2 State giống nhau không 
__eq__(other)
# So sánh fValue có nhỏ hơn fValue khác không 
__lt__(other)
__bool__()
__hash__()
__str__()
```

## Heuristic.py 
_ Import : 
```python
from Cube import *
from ManhattanCube import *
```
_ **class Heuristic** : Cài đặt 3 hàm heuristic, điểm càng nhỏ cho thấy state đó càng tốt 
```python
# Simple heuristic : tính điểm theo số ô cùng màu trên 1 mặt
simpleHeuristic(state)

# Hamming Distance : tính khoảng cách Hamming
hammingDistance(cube)

# Manhattan Distance : tính khoảng cách  Manhattan3 chiều bằng cách  myHeuristic.scoreCube(cube)
manhattanDistance(cube)gọi
```

_ **class myHeuristic** : Sử dụng để tính khoảng cách Manhattan 

## Cube.py 
_ Import : 
```python
import math, random
from copy import copy, deepcopy
from ManhattanCube import*
```

_ Dùng để biểu diễn và sử dụng khối rubik. 

_ Khối rubik được thể hiện dưới dạng ma trận 2D gồm 6 ma trận (n x n) 

_ Cách đánh số các mặt : 

0. Mặt trước 
1. Mặt trên 
2. Mặt phải 
3. Mặt đáy 
4. Mặt trái 
5. Mặt lưng 

_ **class Cube** : 
```python
    #size = kích thước 
    #state = tuples trạng thái
    __init__(n=2, hash=None)

    isSolved()

    # Chỉ dùng 2 move F và U và không vận dụng 2 move liên tiếp trên 1 mặt
    def trueScramble(length):

    # Trả về cách giải rubik bằng cách đảo ngược scramble
    obviousSolution(scramble)

    # Chuyển các move sang ký hiệu
    translateMove(move)

    # Nhận vào số lần scramble
    scramble(length)

    # Quay mặt trước pi/2 và n là số hiệu layer được quay
    turnFront(n)

    turnUp(n)

    turnRight(n)

    turnDown(n)

    turnLeft(n)

    turnBack(n)

    # makeMove nhận vào slice là move và số lần thực hiện move
    makeMove(move)

    # asRows Chuyển 1 mặt thành các mảng theo hàng
    asRows(i)

    # asColumns Chuyển 1 mặt thành các mảng theo cột
    # fashion to asRows
    asColumns(i)

    colToFace(cols)

    rowToFace(rows)

    # reverse Trả về 1 list với thứ tự đảo ngược list ban đầu
    reverse(l)

    rotate(i)

    # rotate layers : dịch phải các phần tử trong list
    rotateLayers(l)

    # in ra trạng thái
    printMap(self)

    # Sinh ra các state con của state cha 
    # 2x : chỉ sử dụng các move F; U; R    
    # all : sử dụng tất cả các move 
    children(self,depth=None)

    # Sao chép trạng thái
    __copy__()

    # hash trạng thái thành 10 số base 6
    __hash__()

    # base 6 encoding with 10 digits
    encode(state)

    # decode an encoded state array
    decode(hash)
```

## ManhattanCube.py 
_ Import : 
```python
from Cube import*
```

_ **class ManhattanCube** : Thể hiện 1 khối rubik dựa vào các góc
```python
__init__(map)
# Trả về thứ tự của góc
findPiece(id)

cublet(a)
```
