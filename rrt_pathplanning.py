import numpy as np
import matplotlib.pyplot as plt


class ReRT:
	#buat class node berisi
	# X, Y, dan XY & parent node sebelumnya
	class Node:
		def __init__(self, x, y):
			self.x = x
			self.y = y
			self.path_x = []
			self.path_y = []
			self.parent = None
	
	#inisialisasi
	def __init__(self, start, finish, obstacles, range, expand_dis, epoch):
		self.start = self.Node(start[0], start[1])
		self.finish = self.Node(finish[0], finish[1])
		self.range = range
		self.expand_dis = expand_dis
		self.epoch = epoch
		self.obstacles = obstacles
		self.node_list = []
	
	#main function di sini
	def path_planning(self):
		#node awal = titik start
		self.node_list = [self.start]
		
		#loop sebanyak epoch
		for i in range(self.epoch):
			print("epoch: " + str(i))
			
			#rnd_node = self.get_np.random_node()
			#generate np.random poin XY dalam interval yang ditentukan
			rnd_node = self.Node(
				np.random.randint(self.range[0], self.range[1]), # np.random x
				np.random.randint(self.range[0], self.range[1])) # np.random y
			
			#plot np.random node baru
			self.draw(rnd=rnd_node, new=None)
			
			#hitung node terdekat dari np.random node yang digenerate
			#hitung dulu jarak euclidean setiap node di list_node
			#dengan np.random node yang baru saja digenerate
			dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
					for node in self.node_list]
		
			#ambil index dari node dengan jarak euclidean terkecil
			min_ind = dlist.index(min(dlist))
			#ambil koordinat XY dari index node terdekat
			nearest_node = self.node_list[min_ind]
			#print("Koordinat node: (%d,%d)" % (nearest_node.x, nearest_node.y))
			
			#generate node baru dari expanding nearest node ke np.random node
			new_node = self.get_new_node(nearest_node, rnd_node, self.expand_dis)
			
			#check apakah node baru aman dan tidak menabrak obstacle
			if self.check_node_safe(new_node):
				print("new node aman")
				#jika aman maka masukkan ke node list
				self.node_list.append(new_node)
				#plot node baru yang sudah dinyatakan aman
				self.draw(rnd=rnd_node, new=new_node)
				
				#cek apakah node baru tersebut sudah mendekati titik finish
				a = new_node.x - self.finish.x
				b = new_node.y - self.finish.y
				c = np.math.hypot(a, b) #jarak terdekat, jarak miring
				
				#dikatakan mendekati titik finish apabila
				#jarak terdekat ke titik finish < jarak jangkauan expand
				if c < self.expand_dis:
					final_node = self.get_new_node(new_node, self.finish, self.expand_dis)
					#tetap harus di cek apakah menyebabkan tabrakan atau tidak
					if self.check_node_safe(final_node):
						#jika aman maka cetak path
						#masukkan dulu titik finish
						path = [[self.finish.x, self.finish.y]]
						#save setiap node dari node list yang sesuai path
						node = self.node_list[len(self.node_list)-1]
						while node.parent is not None:
							path.append([node.x, node.y])
							node = node.parent
						#save koordinat start juga diakhir
						path.append([self.start.x, self.start.y])
						return path
			else:
				print("new node tidak aman")
			#import time
			#time.sleep(1) #buka ini untuk meliihat proses lebih jelas
		#jika sampai habis looping tidak ketemu path maka return None
		return None 
	
	#generate new node
	def get_new_node(self, from_node, to_node, extend_dis):
		#buat object node baru dulu,
		#inisiasi dari nearest node (from node), terserah
		new_node = self.Node(from_node.x, from_node.y)
		new_node.path_x = [new_node.x]
		new_node.path_y = [new_node.y]
		
		#hitung jarak dan sudut antara from node ke to node
		#hitung a (sisi horizontal x)
		a = to_node.x - from_node.x
		#hitung b (sisi vertikal y)
		b = to_node.y - from_node.y
		#hitung c (sisi miring)
		c = np.math.hypot(a, b)

		#jika range extend melebihi panjang sisi miring
		#maka range extend dibuat = sisi miring
		if extend_dis > c:
			extend_dis = c
		
		#bulatkan ke bawah ke bilangan integer terdekat
		n_expand = np.math.floor(extend_dis)
		
		#tentukan koordinat node baru
		#hitung  dulu sudut theta dengan arc tan (b/a)
		t = np.math.atan2(b, a)	
		for _ in range(n_expand):
			#jumlahkan secara berkala sebanyak n_expand
			new_node.x += np.math.cos(t)
			new_node.y += np.math.sin(t)
			#simpan path koordinat XY
			new_node.path_x.append(new_node.x)
			new_node.path_y.append(new_node.y)

		#parent (asal node) dari new node adalah from node
		new_node.parent = from_node

		return new_node
	
	#fungsi cek node baru membuat tabraakan atau tidak
	def check_node_safe(self, node):	
		for (ox, oy, radius) in self.obstacles:
			#hitung jarak XY di path newnode dengan origin koordinat obstacle
			delta_x_list = [(ox - x)**2 for x in node.path_x]
			delta_y_list = [(oy - y)**2 for y in node.path_y]
			#hitung jarak euclidean masing2 titik di path dengan origin koordinat obstacle
			euclid_list = [d_x + d_y for (d_x, d_y) in zip(delta_x_list, delta_y_list)]
			
			#jika ditemukan jarak terdekat berada dalam radius
			# maka terjadi tabrakan antara path dengan obstacle
			if min(euclid_list) < radius**2:
				return False
		#jika tidak maka aman
		return True
	
	#fungsi plot gambar
	def draw(self, rnd=None, new=None):
		#buat frame
		#plt.figure(figsize=(20,20))
		plt.clf()
		
		# stop simulasi = escape
		plt.gcf().canvas.mpl_connect('key_release_event',
			lambda event: [exit(0) if event.key == 'escape' else None])
		
		#gambar np.random node
		if rnd is not None:
			plt.plot(rnd.x, rnd.y, "og")
		#gambar new node yang sudah dinyatakan aman
		if new is not None:
			plt.plot(new.x, new.y, "ob")
		
		#gambar path yang terbentuk
		for node in self.node_list:
			if node.parent:
				plt.plot(node.path_x, node.path_y, "-b")
		
		#gambar obstaclenya
		for (ox, oy, size) in self.obstacles:
			deg = list(range(0, 360, 1)) #tiap 1 derajat
			deg.append(0)
			xl = [ox + size * np.math.cos(np.math.radians(d)) for d in deg]
			yl = [oy + size * np.math.sin(np.math.radians(d)) for d in deg]
			plt.plot(xl, yl, "-r")
		
		#gambar koordinat start dan finish
		plt.plot(self.start.x, self.start.y, "^b")
		plt.plot(self.finish.x, self.finish.y, "^g")
		
		#gambar kotak batasan
		plt.axline((self.range[0], self.range[0]),(self.range[0], self.range[1]), c="k")
		plt.axline((self.range[0], self.range[0]),(self.range[1], self.range[0]), c="k")
		plt.axline((self.range[1], self.range[1]),(self.range[0], self.range[1]), c="k")
		plt.axline((self.range[1], self.range[1]),(self.range[1], self.range[0]), c="k")
		
		#gambar grid dan rangenya
		plt.axis('equal') #grid X dan Y sama
		plt.xticks(np.arange(self.range[0], self.range[1]+1, step=1))
		plt.yticks(np.arange(self.range[0], self.range[1]+1, step=1))
		#batasi range gambarnya
		plt.axis([self.range[0]-1, self.range[1]+1, self.range[0]-1, self.range[1]+1])
		plt.grid(True) #gambar grid
		plt.pause(0.01) #lama penampilan



#MAIN########################
rrt = ReRT(
	start=[2, 2], #point awal
	finish=[13, 13], #point akhir
	range=[0, 15], #range untuk X dan Y
	expand_dis=2, #jarak jelajah
	epoch=500, #jumlah iterasi
	# koordinat x,y, dan radiusnya
	obstacles=[(12, 7, 1), (4, 6, 2), (8, 8, 1), (10, 11, 2), 
				(6, 13, 1.5), (9, 4, 1.5), (3, 11, 1), (13, 4, 1)])

#start routing path
path = rrt.path_planning()

#gambar pathnya
if path is not None:
	print("PATH PLANNING SUCCESS")
	print(path)
	rrt.draw()
	plt.plot([x for (x, y) in path], [y for (x, y) in path], '-g')
	plt.grid(True)
	plt.pause(0.01)
	plt.show()
else:
	print("PATH PLANNING FAILED")
	
	
	
	
	
"""
"""