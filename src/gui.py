#!/usr/bin/env python

import rospy
from grasp_stability_msgs.msg import GraspStability
import gtk
import pygtk


class GraspStabilityControlUI:
    def __init__(self):
        rospy.init_node('grasp_stability_estimator');
        self.pubState = rospy.Publisher('/grasp_stability_estimator/state', GraspStability);
        
        # The window
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.connect("delete_event", self.delete_event)
        self.window.connect("destroy", self.destroy)
        
        # The quit button
        self.button_quit = gtk.Button("Quit")
        self.button_quit.connect("clicked", self.click_quit, None)

        # The publish button
        self.button_publish = gtk.Button("Publish")
        self.button_publish.connect("clicked", self.click_publish, None)
        
        # The context list
        self.context_list = gtk.TreeView(gtk.ListStore(str))
        
        # The quit/publish hbox
        self.hbox2 = gtk.HBox(False, 0)
        self.hbox2.pack_start(self.button_quit, False, False, 5)
        self.hbox2.pack_start(self.button_publish, True, True, 5)
        
        # The list/quit/publish vbox
        self.vbox1 = gtk.VBox(False, 0)
        self.vbox1.pack_start(self.context_list, True, True, 5)
        self.vbox1.pack_start(self.hbox2, False, False, 5)
        
        # The grasp cat buttons
        self.button_graspcat_good = gtk.ToggleButton("Good grasp")
        self.button_graspcat_medium = gtk.ToggleButton("Medium grasp")
        self.button_graspcat_bad = gtk.ToggleButton("Bad grasp")
        self.button_graspcat_good.connect("clicked", self.click_graspcat, None)
        self.button_graspcat_medium.connect("clicked", self.click_graspcat, None)
        self.button_graspcat_bad.connect("clicked", self.click_graspcat, None)
        
        # Text boxes and labels
        self.hbox_conf = gtk.HBox(False, 0)
        self.lbl_graspconf = gtk.Label("Confidence: ")
        #self.txt_graspconf = gtk.EditField("")
        self.hbox_conf.pack_start(self.lbl_graspconf, False, False, 5)
        self.hbox_qual = gtk.HBox(False, 0)
        self.lbl_qual = gtk.Label("Quality: ")
        #self.txt_graspconf = gtk.EditField("")
        self.hbox_qual.pack_start(self.lbl_qual, False, False, 5)
        
        # The settings vbox
        self.vbox2 = gtk.VBox(False, 0)
        self.vbox2.pack_start(self.button_graspcat_good, False, False, 5)
        self.vbox2.pack_start(self.button_graspcat_medium, False, False, 5)
        self.vbox2.pack_start(self.button_graspcat_bad, False, False, 5)
        self.vbox2.pack_start(self.hbox_conf, False, False, 5)
        self.vbox2.pack_start(self.hbox_qual, False, False, 5)
        
        # The main hbox
        self.hbox1 = gtk.HBox(False, 0)
        self.hbox1.pack_start(self.vbox1, True, True, 5)
        self.hbox1.pack_start(self.vbox2, True, True, 5)
        
        self.window.add(self.hbox1)
        
        self.window.show_all()
        
        self.window.set_title("Grasp Stability Estimation Dummy UI")
        self.window.resize(640, 480)
        
        gtk.main()
        
        pass;
    
    def delete_event(self, widget, event, data=None):
        return False
    
    def quit(self):
        gtk.main_quit()
    
    def click_quit(self, widget, data=None):
        self.quit()

    def click_publish(self, widget, data=None):
        grasp_cat = 0
        if self.button_graspcat_good.get_active():
            grasp_cat = 1
        elif self.button_graspcat_medium.get_active():
            grasp_cat = 2
        elif self.button_graspcat_bad.get_active():
            grasp_cat = 3
        
        grasp_quality = 0.95
        estimation_confidence = 0.73
        measurement_context_id = 'test-id'
        
        rospy.sleep(0.1)
        self.publish(grasp_cat, grasp_quality, estimation_confidence, measurement_context_id)

    def click_graspcat(self, widget, data=None):
        if widget == self.button_graspcat_good:
            if self.button_graspcat_good.get_active():
                self.button_graspcat_medium.set_active(False)
                self.button_graspcat_bad.set_active(False)

        if widget == self.button_graspcat_medium:
            if self.button_graspcat_medium.get_active():
                self.button_graspcat_good.set_active(False)
                self.button_graspcat_bad.set_active(False)

        if widget == self.button_graspcat_bad:
            if self.button_graspcat_bad.get_active():
                self.button_graspcat_good.set_active(False)
                self.button_graspcat_medium.set_active(False)
    
    def destroy(self, widget, data=None):
        self.quit()

    def publish(self, category, quality, confidence, context):
        gsPublish = GraspStability()
        gsPublish.grasp_quality = quality
        gsPublish.estimation_confidence = confidence
        gsPublish.grasp_category = category
        gsPublish.measurement_context_id = context
        
        self.pubState.publish(gsPublish)


if __name__ == '__main__':
    gcuMain = GraspStabilityControlUI()
