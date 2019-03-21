# coding: utf-8

import sys
import numpy as np
import math
import os

##############################################################################
# CHANNEL
##############################################################################
class Channel( object ):
	__slots__=[ 'index','motion', 'matrix' ]
	def __init__( self, index = 0 ):
		self.index = index
		self.motion = []
		self.matrix = np.identity(4)

class ZeroChannel( Channel ):
	__slots__ = []

# Position
class ChannelPositionX( Channel ):
	__slots__ = []
	def __str__( self ):
		return "px"

class ChannelPositionY( Channel ):
	__slots__ = []
	def __str__( self ):
		return "py"

class ChannelPositionZ( Channel ):
	__slots__ = []
	def __str__( self ):
		return "pz"

# Rotation
class ChannelRotationX( Channel ):
	__slots__ = []
	def __str__( self ):
		return "rx"

class ChannelRotationY( Channel ):
	__slots__ = []
	def __str__( self ):
		return "ry"

class ChannelRotationZ( Channel ):
	__slots__ = []
	def __str__( self ):
		return "rz"

channel_map = {
	'Xposition': ChannelPositionX,
	'Yposition': ChannelPositionY,
	'Zposition': ChannelPositionZ,
	'Xrotation': ChannelRotationX,
	'Yrotation': ChannelRotationY,
	'Zrotation': ChannelRotationZ,
}

def createChannel( key, index ):
	if key in channel_map:
		return channel_map[ key ]( index )
	raise ValueError("unknown key: %s" %key )


##############################################################################
# JOINT
##############################################################################
class Joint(object):
	__slots__=[ 'name', 'parent', 'children', 'offset', 'tail', 'channels', 'channelMap' ]

	def __init__( self, name ):
		self.name = name
		self.parent = None
		self.children = []
		self.offset = None
		self.channels = []
		self.channelMap = {
				ChannelPositionX: ZeroChannel(),
				ChannelPositionY: ZeroChannel(),
				ChannelPositionZ: ZeroChannel(),
				ChannelRotationX: ZeroChannel(),
				ChannelRotationY: ZeroChannel(),
				ChannelRotationZ: ZeroChannel(),
				}

	def addChannel( self, channel ):
		self.channels.append( channel )
		self.channelMap[ channel.__class__ ] = channel

	def addChild( self, child ):
		self.children.append( child )
		child.parent = self
		return child

	def show(self, indent=''):
		channels=' '.join(str(c) for c in self.channels)
		if self.name:
			print self.name
		for child in self.children:
			child.show(indent+'  ')

##############################################################################
# LOADER
##############################################################################
class Loader( object ):
	__slots__ = [ 'root', 'channels', 'frame_count', 'frame_interval', 'joint_list' ]

	def __init__( self ):
		self.joint_list = []

	def load( self, path ):
		io = open( path, "rb" )
		if not io:
			raise "fail to open %s" %path
		return self.process( io )

	# Load joint and motion data
	def process( self, io ):
		self.channels = []
		if io.readline().strip() != "HIERARCHY":
			raise "invalid signature"
		type, name = io.readline().strip().split()
		self.root = Joint( name )
		self.joint_list.append( self.root )
		self.parseJoint( io, self.root )
		self.parseMotion( io )

	# Register Joint data
	def parseJoint( self, io, joint ):
		line = io.readline().strip()
		if line != "{":
			raise "no {"

		type, x, y, z = io.readline().strip().split()
		if type != "OFFSET":
			raise "no OFFSET"
		joint.offset = [ float(x), float(y), float(z) ]

		tokens=io.readline().strip().split()
		if tokens.pop(0) != "CHANNELS":
			raise "no CHANNELS"

		channelCount = int( tokens.pop(0) )
		joint.channels = []
		for channel in tokens:
			channel = createChannel( channel, len( self.channels ) )
			joint.addChannel( channel )
			self.channels.append( channel )
		assert( len( joint.channels ) == channelCount )

		# Check number of channels
		if joint.parent:
			assert( channelCount == 3 )
		else:
			assert( channelCount == 6 )

		# Register channels
		while True:
			line = io.readline()
			if line == "":
				raise "invalid eof"
			tokens = line.strip().split()
			if tokens[0] == "JOINT":
				child = joint.addChild( Joint( tokens[1] ) )
				self.joint_list.append( child )
				self.parseJoint( io, child )
			elif tokens[0] == "End":
				line = io.readline().strip()
				if line != "{":
					raise "no {"
				type, x, y, z = io.readline().strip().split()
				if type != "OFFSET":
					raise "no OFFSET"
				joint.tail = [ float(x), float(y), float(z) ]
				if io.readline().strip() != "}":
					raise "no }"
			elif tokens[0] == "}":
				return
			else:
				raise "unknown type"

	# Input motion data
	def parseMotion( self, io ):
		line = io.readline().strip()
		if line != "MOTION":
			raise "no MOTION"
		type, frame_count = io.readline().strip().split()
		if type != "Frames:":
			raise "no Frames:"
		tokens = io.readline().strip().split()
		if tokens[0] != "Frame":
			raise "no Frame"
		if tokens[1] != "Time:":
			raise "no Time:"
		self.frame_count = int( frame_count )
		self.frame_interval = float( tokens[2] )

		# Input motion data per frame
		while True:
			line = io.readline()
			if line == "":
				break
			tokens = line.strip().split()
			assert( len( tokens ) == len( self.channels ) )
			for i,t in enumerate( tokens ):
				self.channels[i].motion.append( float(t) )

def load( path ):
	l = Loader()
	try:
		l.load( path )
	except Exception as e:
		print e
		return 
	return l

###############################################################################
if __name__=='__main__':
	l = load(filename)