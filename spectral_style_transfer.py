#coding:UTF-8
 
import numpy as np
import scipy
from scipy.fftpack import fft, ifft
import math
from scipy.optimize import minimize
import scipy
import copy
import os
import time
import bvhwriter2
import bvhloader3
import dynamic_time_warping5


frame_num = 3200                       # data quantity
samp_num =  17
samp_half = 9
dim = 87

count = np.arange( samp_num )
w_frame = np.arange( samp_half )
num = count - samp_half

w1 = ( 1 + np.cos(( w_frame * math.pi ) / ( samp_half - 1 ))) / 2
w2 = ( 1 + np.cos((( w_frame - samp_half + 1 ) * math.pi ) / ( samp_half - 1 ))) / 2

hamming_window = np.full( samp_num, 1.0 )
inverse_window = np.full( samp_num, 1.0 )

opt_R = np.zeros(( dim, samp_half ), dtype = float )
opt_A = np.zeros(( dim, samp_half ), dtype = float )
port = 0

source_file = 'MotionData/Strong_b_1_n_s.bvh'
reference_file = 'MotionData/Gait_1_n_s2.bvh'
target_file = 'MotionData/Jump_a_1_n2.bvh'
retarget_file = 'retarget.bvh'

# Objective function
# sum(R'-StyleValue)
def func( x, style, lambda_s, lambda_a, lambda_b, pre_retarget_R, target_A, joint_count, angle_const ):
    first = sum(( x[:samp_half] - style ) ** 2 )
    second = lambda_s * sum(( x[:samp_half] - pre_retarget_R )**2)
    third = lambda_a * sum(( x[samp_half:] - target_A ) ** 2 )
    f = first + second + third
    for k in xrange( joint_count ):
        f +=  lambda_b * sum(( x[samp_half:]  - angle_const[k][:])**2 )
    return f

def func_deriv( x, style, lambda_s, lambda_a, lambda_b, pre_retarget_R, target_A, joint_count, angle_const ):
    dfdx = np.zeros( samp_num + 1 )
    dfdx[:samp_half] = ( 2 * ( x[:samp_half] - style ) + lambda_s*2*( x[samp_half:] - target_A ) )
    dfdx[samp_half:] = ( lambda_a * 2 *  ( x[samp_half :] - target_A ) )
    for k in xrange( joint_count ):
        dfdx[samp_half:]  += (  lambda_b  * 2 * ( x[samp_half:]  - angle_const[k][:] ))
    return dfdx

def myFFT(y):
    shift_yf = np.zeros( samp_num, dtype = complex )
    yf = fft(y)
    shift_yf[:samp_half - 1] = yf[samp_half:]
    shift_yf[samp_half - 1:] = yf[:samp_half]
    return shift_yf

def myIFFT(yf):
    shift_yf = np.zeros( samp_num , dtype = complex )
    shift_yf[:samp_half] = yf[samp_half - 1 :]
    shift_yf[samp_half:] = yf[:samp_half - 1]
    y = ifft(shift_yf)
    return y

def dft( y, tf ):
    cut_y = y[ tf - samp_half + 1 : tf + samp_half  ]
    # Sliding Window function
    samp_y = cut_y * hamming_window
    samp_yf = myFFT(samp_y)
    # Fourier transfer
    cut_yf = samp_yf[:samp_half ] / samp_half
    return cut_yf

def transfer( tf, source_joint, reference_joint, target_joint, retarget_joint ):
    global opt_R, opt_A, port
    for channel_num in xrange( len( target_joint.channels )):
        # Read each waves
        source_y = source_joint.channels[channel_num].motion[:frame_num]
        reference_y = reference_joint.channels[channel_num].motion[:frame_num]
        target_y = target_joint.channels[channel_num].motion[:frame_num]

        # Set sampling source
        source_yf = dft( source_y, tf )
        source_R = np.abs( source_yf )
        source_A = np.angle( source_yf )
        
        # Set sampling reference 
        reference_yf = dft( reference_y, tf )
        reference_R = np.abs( reference_yf )
        
        # Set sampling target
        target_yf = dft( target_y, tf )
        target_R = np.abs( target_yf )
        target_A = np.angle( target_yf )

        # Set initial value
        x0 = np.zeros( samp_num + 1 )
        if tf == samp_half-1:
            x0 = np.random.rand( samp_num + 1 )
            pre_retarget_R = np.full( samp_half , 0.0 )
        else:
            pre_retarget_R = opt_R[port][:]
            x0[:samp_half] = opt_R[port][:]
            x0[samp_half:] = opt_A[port][:]

        # Set normalization factor
        if tf == samp_half - 1:
            lambda_s = 0.0
        else:
            lambda_s = 0.01
        lambda_a = np.mean( target_R ) / ( math.pi / 2 )
        lambda_b = np.mean( target_R ) / ( math.pi / 2 )

        # Calculate StyleValue
        s = np.zeros( samp_half )
        if np.nanmax( target_R ) != 0:
            s = target_R / np.nanmax( target_R )
        style = target_R + s * ( source_R - reference_R )

        # Set Angle data
        joint_count = 0
        angle_const = []

        if retarget_joint.parent:
            parent = retarget_joint.parent.channels[channel_num].__class__.__name__
            parent_num = channel_num
            if parent != "ChannelPositionX" and parent != "ChannelPositionY" and parent != "ChannelPositionZ":
                angle_list = np.full( samp_half , 0.0 )
                joint_count += 1
                parent_retarget_yf = dft( retarget_joint.parent.channels[parent_num].motion, tf )
                angle_list += np.angle( parent_retarget_yf )
                parent_source_yf = dft( source_joint.parent.channels[parent_num].motion, tf )
                angle_list -= np.angle( parent_source_yf )
                angle_list +=  source_A
                angle_const.append(angle_list)

        if retarget_joint.children:
            for joint_num in xrange ( len( retarget_joint.children )):
                angle_list = np.full( samp_half , 0.0 )
                joint_count += 1
                child_retarget_yf = dft( retarget_joint.children[joint_num].channels[channel_num].motion, tf )
                angle_list += np.angle( child_retarget_yf )
                child_source_yf = dft( source_joint.children[joint_num].channels[channel_num].motion, tf )
                angle_list -= np.angle( child_source_yf )
                angle_list +=  source_A
                angle_const.append(angle_list)

        # Optimization method
        arg = ( style, lambda_s, lambda_a, lambda_b, pre_retarget_R, target_A, joint_count, angle_const )
        res = minimize( func, x0, args = arg, method = 'CG', jac = func_deriv, tol = 1e-15) 

        opt_R[port][:] = res.x[:samp_half]
        opt_A[port][:] = res.x[samp_half:]

        samp_retarget_yf_list = []
        for k in xrange( samp_half ):
            samp_retarget_yf_list.append( res.x[ k ] * math.cos(res.x[ samp_half + k ]) - res.x[ k ] * math.sin( res.x[ samp_half + k ]) * 1j )
        for k in xrange( samp_half - 1 ):
            samp_retarget_yf_list.append( res.x[ samp_half - k - 2 ] * math.cos(res.x[ samp_num - k - 1 ]) + res.x[ samp_half - k - 2 ] * math.sin(res.x[ samp_num - k - 1 ]) * 1j )
        samp_retarget_yf =  np.array( samp_retarget_yf_list ) * samp_half
        samp_retarget_y = np.real( myIFFT( samp_retarget_yf )) 
        samp_retarget_y *= inverse_window


        if tf == samp_half-1:
            samp_retarget_y[:samp_half] *= w2
            retarget_joint.channels[channel_num].motion[tf - samp_half + 1 : tf + samp_half ] = samp_retarget_y
        else:
            pre_y =  retarget_joint.channels[channel_num].motion[tf - samp_half + 1 : tf + 1 ]
            pre_y *= w1
            samp_retarget_y[:samp_half] *= w2
            samp_retarget_y[:samp_half] +=pre_y
            retarget_joint.channels[channel_num].motion[tf - samp_half + 1 : tf + samp_half ] = samp_retarget_y
        port = port + 1

# Optimize for each joint
def separate( tf, source, reference, target, retarget ):
    for child_num in xrange( len(target.children) ):
        transfer( tf, source.children[child_num], reference.children[child_num], target.children[child_num], retarget.children[child_num] )
        if target.children[child_num].children:
            separate( tf, source.children[child_num], reference.children[child_num], target.children[child_num], retarget.children[child_num] )

def main():
    startTime = time.time()
    global frame_num, port
    print w1
    print w2

    source_load = bvhloader3.load( source_file )
    #source_load = bvhloader3.load( 'dtw_source.bvh' )
    source_root = source_load.root
    source_frame = source_load.frame_count
    frame_num = source_frame
    print "source file : ",source_frame,"frames"

    reference_load = bvhloader3.load( reference_file )
    #reference_load = bvhloader3.load(  'dtw_reference.bvh' )
    reference_root = reference_load.root
    print "reference file : ",reference_load.frame_count,"frames"

    target_load = bvhloader3.load( target_file )
    target_root = target_load.root
    target_frame = target_load.frame_count
    print "target file : ",target_frame,"frames"
    
    #frame_num = target_frame

    #dynamic_time_warping5.timeWarp( target_load, source_load )
    dynamic_time_warping5.timeWarp( source_load, reference_load )
    #bvhwriter2.bvhWrite( 'dtw_source.bvh', source_load, frame_num )
    bvhwriter2.bvhWrite( 'dtw_reference.bvh', reference_load, frame_num )

    retarget_load = copy.deepcopy( target_load )
    retarget_root = retarget_load.root
    
    global haming_window, inverse_window
    hamming_window = np.hamming( samp_num )
    inverse_window = 1.0/hamming_window
    for tf in xrange( samp_half - 1, frame_num - samp_half + 1, samp_half - 1 ):
        print tf
        port = 0
        separate( tf, source_root, reference_root, target_root, retarget_root )

    bvhwriter2.bvhWrite( retarget_file, retarget_load, frame_num )
    endTime = time.time()
    print "Time:", round(( endTime - startTime ), 2 ),"sec"

if __name__ == '__main__':
    main()