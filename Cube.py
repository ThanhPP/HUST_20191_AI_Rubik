import math, random
from copy import copy, deepcopy
from ManhattanCube import*

class Cube:

    def __init__(self, n=2, hash=None):
        self.size = n
        self.state = [[],[],[],[],[],[]]
        for i in range(6):
            for j in range(n*n):
                self.state[i].append(i)
        if n % 2 == 0:
            self.moves = (3 * n) - 1
        elif n % 2 == 1:
            self.moves = (3 * (n - 1)) - 1

    def isSolved(self):
        for i in range(6):
            for j in range((self.size * self.size) - 1):
                if self.state[i][j] != self.state[i][j + 1]:
                    return False
        return True

    def trueScramble(self, length):
        moves = []
        for i in range(length):
            if i == 0:
                move = (random.randint(0,2), random.randint(1,3))
                self.makeMove(move)
                moves.append(move)
            else:
                move = (random.randint(0, 2), random.randint(1, 3))
                while move[0] == moves[i - 1][0]:
                    move = (random.randint(0, 2), random.randint(1, 3))
                self.makeMove(move)
                moves.append(move)
        return moves

    def obviousSolution(scramble):
        scramble.reverse()
        solution = []
        for i in scramble:
            solution.append((i[0], 4 - i[1]))
        return solution

    def translateMove(move):
        if move == None:
            return None
        if move[0] == 0:
            answer = "F"
        if move[0] == 1:
            answer = "U"
        if move[0] == 2:
            answer = "R"
        if move[0] == 3:
            answer = "D"
        if move[0] == 4:
            answer = "L"
        if move[0] == 5:
            answer = "B"
        if move[1] == 2:
            answer = answer + "2"
        if move[1] == 3:
            answer = answer + "'"
        return answer

    def scramble(self, length):
        moves = []
        for i in range(length):
            move = (random.randint(0, 4), random.randint(1, 3))
            while i >= 1 and (moves[i - 1][0] == move[0] or move[0] == self.opposite(moves[i - 1][0])):
                move = (random.randint(0, 4), random.randint(1, 3))
            moves.append(move)
            self.makeMove(move)
        return moves

    def opposite(i):
        if i == 0:
            return 5
        if i == 1:
            return 3
        if i == 2:
            return 4
        if i == 3:
            return 1
        if i == 4:
            return 2
        if i == 5:
            return 0

    def turnFront(self, n):
        assert n <= self.size/2
        rotation = [self.asRows(1)[self.size - n - 1], self.asColumns(2)[n], self.asRows(3)[n], self.asColumns(4)[self.size - 1 - n]]
        self.rotateLayers(rotation)
        # repair Up
        temp = self.asRows(1)
        temp[self.size - 1 - n] = self.reverse(rotation[0])
        self.state[1] = self.rowToFace(temp)
        # repair Right
        temp = self.asColumns(2)
        temp[n] = rotation[1]
        self.state[2] = self.colToFace(temp)
        # repair Down
        temp = self.asRows(3)
        temp[n] = self.reverse(rotation[2])
        self.state[3] = self.rowToFace(temp)
        # repair Left
        temp = self.asColumns(4)
        temp[self.size - 1 - n] = rotation[3]
        self.state[4] = self.colToFace(temp)

        if n == 0:
            self.rotate(0)

    def turnUp(self, n):
        assert n <= self.size/2
        rotation = [self.asRows(0)[n], self.asRows(4)[n], self.asRows(5)[self.size - n - 1], self.asRows(2)[n]]
        self.rotateLayers(rotation)

        temp = self.asRows(0)
        temp[n] = rotation[0]
        self.state[0] = self.rowToFace(temp)

        temp = self.asRows(4)
        temp[n] = rotation[1]
        self.state[4] = self.rowToFace(temp)

        temp = self.asRows(5)
        temp[self.size - 1 - n] = self.reverse(rotation[2])
        self.state[5] = self.rowToFace(temp)

        temp = self.asRows(2)
        temp[n] = self.reverse(rotation[3])
        self.state[2] = self.rowToFace(temp)


        if n == 0:
            self.rotate(1)


    def turnRight(self, n):
        assert n <= self.size/2
        rotation = [self.asColumns(0)[self.size - 1 - n], self.asColumns(1)[self.size - n - 1], self.asColumns(5)[self.size - n - 1], self.asColumns(3)[self.size - n - 1]]
        self.rotateLayers(rotation)

        temp = self.asColumns(0)
        temp[self.size - 1 - n] = rotation[0]
        self.state[0] = self.colToFace(temp)

        temp = self.asColumns(1)
        temp[self.size - 1 - n] = rotation[1]
        self.state[1] = self.colToFace(temp)

        temp = self.asColumns(5)
        temp[self.size - 1 - n] = rotation[2]
        self.state[5] = self.colToFace(temp)

        temp = self.asColumns(3)
        temp[self.size - 1 - n] = rotation[3]
        self.state[3] = self.colToFace(temp)


        if n == 0:
            self.rotate(2)


    def turnDown(self, n):
        assert n <= self.size/2
        rotation = [self.asRows(0)[self.size - n - 1], self.asRows(2)[self.size - n - 1], self.asRows(5)[n], self.asRows(4)[self.size - n - 1]]
        self.rotateLayers(rotation)

        temp = self.asRows(0)
        temp[self.size - 1 - n] = rotation[0]
        self.state[0] = self.rowToFace(temp)

        temp = self.asRows(2)
        temp[self.size - 1 - n] = rotation[1]
        self.state[2] = self.rowToFace(temp)

        temp = self.asRows(5)
        temp[n] = self.reverse(rotation[2])
        self.state[5] = self.rowToFace(temp)

        temp = self.asRows(4)
        temp[self.size - 1 - n] = self.reverse(rotation[3])
        self.state[4] = self.rowToFace(temp)


        if n == 0:
            self.rotate(3)


    def turnLeft(self, n):
        assert n <= self.size/2
        rotation = [self.asColumns(0)[n], self.asColumns(3)[n], self.asColumns(5)[n], self.asColumns(1)[n]]
        self.rotateLayers(rotation)

        temp = self.asColumns(0)
        temp[n] = rotation[0]
        self.state[0] = self.colToFace(temp)

        temp = self.asColumns(3)
        temp[n] = rotation[1]
        self.state[3] = self.colToFace(temp)

        temp = self.asColumns(5)
        temp[n] = rotation[2]
        self.state[5] = self.colToFace(temp)

        temp = self.asColumns(1)
        temp[n] = rotation[3]
        self.state[1] = self.colToFace(temp)

        if n == 0:
            self.rotate(4)


    def turnBack(self, n):
        assert n <= self.size/2
        rotation = [self.asRows(1)[n], self.asColumns(4)[n], self.asRows(3)[self.size - n - 1], self.asColumns(2)[self.size - n - 1]]
        self.rotateLayers(rotation)

        temp = self.asRows(1)
        temp[n] = self.reverse(rotation[0])
        self.state[1] = self.rowToFace(temp)

        temp = self.asColumns(4)
        temp[n] = rotation[1]
        self.state[4] = self.colToFace(temp)

        temp = self.asRows(3)
        temp[self.size - 1 - n] = self.reverse(rotation[2])
        self.state[3] = self.rowToFace(temp)

        temp = self.asColumns(2)
        temp[self.size - 1 - n] = rotation[3]
        self.state[2] = self.colToFace(temp)

        if n == 0:
            self.rotate(5)


    def makeMove(self, move):
        for i in range(move[1]):
            if move[0] % 6 == 0:
                self.turnFront(int(move[0]/6))
            if move[0] % 6 == 1:
                self.turnUp(int((move[0] - 1)/6))
            if move[0] % 6 == 2:
                self.turnRight(int((move[0] - 2)/6))
            if move[0] % 6 == 3:
                self.turnDown(int((move[0] - 3)/6))
            if move[0] % 6 == 4:
                self.turnLeft(int((move[0] - 4) / 6))
            if move[0] % 6 == 5:
                self.turnBack(int((move[0] - 5) / 6))
        return self

    def asRows(self, i):
        rows = []
        for j in range(self.size):
            row = []
            for f in range((self.size * j),(self.size * (j + 1))):
                row.append(self.state[i][f])
            rows.append(row)
        return rows

    def asColumns(self, i):
        cols = []
        for j in range(self.size):
            col = []
            for f in range(self.size):
                col.append(self.state[i][(f * self.size) + j])
            cols.append(col)
        return cols

    def colToFace(self,cols):
        l = []
        for i in range(self.size):
            for j in range(self.size):
                l.append(cols[j][i])
        return l

    def rowToFace(self, rows):
        l = []
        for i in range(self.size):
            for j in range(self.size):
                l.append(rows[i][j])
        return l

    def reverse(l):
        j = []
        for i in range(len(l)):
            j.append(l[len(l) - i - 1])
        return j

    def rotate(self, i):
        self.state[i] = self.colToFace(self.reverse(self.asRows(i)))

    def rotateLayers(l):
        temp = l[0]
        l[0] = l[3]
        l[3] = l[2]
        l[2] = l[1]
        l[1] = temp
        return l

    def printMap(self):
        for i in range(6):
            if i == 0:
                print("0 ~ Front")
            if i == 1:
                print("1 ~ Up")
            if i == 2:
                print("2 ~ Right")
            if i == 3:
                print("3 ~ Down")
            if i == 4:
                print("4 ~ Left")
            if i == 5:
                print("5 ~ Back")
            rows = self.asRows(i)
            for j in rows:
                print(j)

    def children(self,depth=None):
        children = []
        if depth == '2x':
            for i in range(3):
                children.append(((i,1),self.__copy__().makeMove((i,1))))
                children.append(((i, 2), self.__copy__().makeMove((i, 2))))
                children.append(((i,3),self.__copy__().makeMove((i,3))))
            return children

        for i in range(self.moves):
            children.append(((i,1),self.__copy__().makeMove((i,1))))
            if depth == 'all' or depth == 'prime':
                children.append(((i,3),self.__copy__().makeMove((i,3))))
            if depth == 'all' or depth == 'double':
                children.append(((i,2), self.__copy__().makeMove((i,2))))
        return children

    def __copy__(self):
        m = Cube(self.size)
        m.state = deepcopy(self.state)
        return m

    def __hash__(self):
        return Cube.encode(self.state)


    def encode(state):
        encoding = 0
        count = 0
        for i in state:
            for j in i:
                encoding += j*(6**count)
                count += 1
        return encoding

    @staticmethod
    def decode(hash):
        hash_10 = hash
        state = [[],[],[],[],[],[]]
        num_face_cublets = math.ceil(math.log(hash_10, 6))//6

        for i in range(6):
            for j in range(num_face_cublets):
                state[i].append(int(hash_10%6))
                hash_10 = int(hash_10//6)
        return state


if __name__ == '__main__':
    cube = Cube(2)
    print(cube.scramble(5))
    n = ManhattanCube(cube)
    print(n.cube)

'''
Map key:
0 -> Front
1 -> Up
2 -> Right
3 -> Down
4 -> Left
5 -> Back
'''