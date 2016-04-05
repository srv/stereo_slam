#!/usr/bin/env python
import roslib; roslib.load_manifest('stereo_slam')
import rospy
import pylab
import math
import numpy as np
import string
import weakref
import time
import ntpath
import os
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Odometry

# Global variables
ax = None
ax_list = []

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass


def check_file_len(file):
  """ Check if the file length is > 0 """
  f = open(file)
  lines = f.readlines()
  f.close()
  return len(lines) > 0


def rm_ax(ax_id):
  """ Remove the axes lines """
  if (ax_id is not None and ax_id):
    l = ax_id.pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l


def odometry_callback(odom_msg, filepath):
  """ Callback to save the odometry data into a file """

  # Extract the data
  line = ( str(odom_msg.header.stamp.secs) +
           str(odom_msg.header.stamp.nsecs) + ',' +
           str(odom_msg.header.seq) + ',' +
           str(odom_msg.header.stamp.secs) +
           str(odom_msg.header.stamp.nsecs) + ',' +
           str(odom_msg.child_frame_id) + ',' +
           str(odom_msg.header.frame_id) + ',' +
           str(odom_msg.pose.pose.position.x) + ',' +
           str(odom_msg.pose.pose.position.y) + ',' +
           str(odom_msg.pose.pose.position.z) + ',' +
           str(odom_msg.pose.pose.orientation.x) + ',' +
           str(odom_msg.pose.pose.orientation.y) + ',' +
           str(odom_msg.pose.pose.orientation.z) + ',' +
           str(odom_msg.pose.pose.orientation.w) + '\n')

  # Append the data to the file
  with open(filepath, "a") as odomfile:
      odomfile.write(line)


def real_time_plot(files):
  """ Function to plot the data saved into the files in real time """
  global ax, ax_list

  # Defines
  graph_vertices_file = 'graph_vertices.txt'
  graph_edges_file = 'graph_edges.txt'

  # Define colors
  colors = ['g','r','b','k','c','m','y']
  linewidth = 2.0

  # Remove all previous axes
  for axes in ax_list:
    rm_ax(axes)

  ax_list = []
  i_color = 0
  for filename in files:

    # Check if file exists
    if (filename != "" and os.path.exists(filename) and check_file_len(filename)):

      # Get the file contents
      try:
        data = pylab.loadtxt(filename, delimiter=',', comments='%', usecols=(5,6,7))
      except Exception, e:
        rospy.loginfo("Impossible to read the file %s", filename)
        return

      # Check dimension
      if (len(data.shape) == 1):
        data = np.array([data])

      # Get legend
      base_name = os.path.basename(filename)
      base_name = os.path.splitext(base_name)
      base_name = base_name[0]

      # Plot
      ax_tmp = ax.plot(data[:,0], data[:,1], data[:,2], colors[i_color], linewidth=linewidth, label=base_name)
      ax_list.append(ax_tmp)
      i_color = i_color + 1

      # Update color
      if i_color >= len(colors):
        i_color = 0;

  # Plot the graph vertices and edges
  if (os.path.exists(graph_vertices_file) and check_file_len(graph_vertices_file)):

    try:
      data = pylab.loadtxt(graph_vertices_file, delimiter=',', usecols=(5,6,7))
    except Exception, e:
      rospy.loginfo("Impossible to read the file %s", graph_vertices_file)
      return

    if (len(data.shape) == 1):
      data = np.array([data])
    ax_tmp = ax.plot(data[:,0], data[:,1], data[:,2], colors[i_color], linewidth=linewidth, label='Stereo Slam', marker='o')
    ax_list.append(ax_tmp)

  if (os.path.exists(graph_edges_file) and check_file_len(graph_edges_file)):

    try:
      data = pylab.loadtxt(graph_edges_file, delimiter=',', usecols=(2,3,4,9,10,11))
    except Exception, e:
      rospy.loginfo("Impossible to read the file %s", graph_edges_file)
      return

    if (len(data.shape) == 1):
      data = np.array([data])
    for i in range(len(data)):
      vect = []
      vect.append([data[i,0], data[i,1], data[i,2]])
      vect.append([data[i,3], data[i,4], data[i,5]])
      vect =  np.array(vect)
      ax_tmp = ax.plot(vect[:,0], vect[:,1], vect[:,2], colors[i_color], linewidth=linewidth-1, linestyle='--')
      ax_list.append(ax_tmp)

  # Update the plot
  pyplot.draw()

  # Show legend
  if (ax_list):
    ax.legend_ = None
    ax.legend()


def plot_odom(list_to_plot):
  """ Main node """
  global ax

  # Setup the font for the graphics
  font = {'family' : 'Sans',
          'weight' : 'normal',
          'size'   : 16}
  pylab.rc('font', **font)

  # Init figure
  fig = pylab.figure(1)
  ax = Axes3D(fig)
  ax.grid(True)
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.set_zlabel("Z")

  # Determine if user enters files or topics
  list_of_files = []
  for to_plot in list_to_plot:
    if os.path.exists(to_plot) is False:

      # It is a topic, create the file
      filename = to_plot
      if (to_plot[0] == '/'):
        filename = filename[1:]
      filepath = os.path.join(os.path.expanduser('~'), '.ros', filename.replace('/', '_') + '.txt')
      f = open(filepath, 'w')
      f.close();

      # Add this file to the list to be plotted
      list_of_files.append(filepath)

      # Create the subscriber
      rospy.Subscriber(to_plot,
                       Odometry,
                       odometry_callback,
                       callback_args = filepath,
                       queue_size = 1)

    else:
      list_of_files.append(to_plot)


  # Start timer for real time plot
  timer = fig.canvas.new_timer(2000)
  real_time_plot(list_of_files)
  timer.add_callback(real_time_plot, list_of_files)
  timer.start()
  pylab.show()


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(
          description='Plot 3D graphics of odometry data files (.txt) or ros topics in real time.',
          formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('list_to_plot', metavar='L', nargs='+',
                   help='list of files or topics to be plotted')
  args = parser.parse_args()
  list_to_plot = args.list_to_plot

  try:
      # Init node
      rospy.init_node('plot_odom')
      plot_odom(list_to_plot)
      rospy.spin()
  except rospy.ROSInterruptException:
      pass