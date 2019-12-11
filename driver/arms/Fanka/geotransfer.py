import numpy as np

class Rotation():
	def __init__(self, arg):
		self.is_r = True
		self.matrix = []
		self.eular = []
		self.quaternion = []
		self.axis_angle = []

		if self.is_matrix(arg):
			self.matrix = np.matrix(arg)
		elif self.is_eular(arg):
			pass 
		elif self.is_quaternion(arg):
			pass
		elif self.is_axis_angle(arg):
			pass
		else:
			print('fail')
		# Matrix, Euler, Quaternion, Axis-Angle

	def __mul__(self,r):
		if not hasattr(r,'is_r'):
			print('must mul an INSTANCE ROTATION!')
			return 
		return Rotation(self.matrix * r.matrix)

	def is_matrix(self,arg):
		if type(arg) is type(np.matrix([])) or type(arg) is type(np.array([])) or type(arg) is list: # numpy class
			arg = np.matrix(arg)
		x,y = arg.shape
		if (x==1 and y==9) or (x==1 and y==16) or (x==3 and y==3) or (x==4 and y==4):
			return True
		else:
			return False


	def is_eular(self,arg):
		pass

	def eular(self,axes='sxyz'):
		pass
	def matrix(self):
		pass
	def quaternion(self,):
		pass
	def axis_angle(self,):
		pass

if __name__ == '__main__':
	a = np.matrix([1,2,3,4,5,6,7,8,9])
	x, y = a.shape
	print(x,y)
	a.resize((3,3))
	print(a,type(a) is type(np.matrix([])))