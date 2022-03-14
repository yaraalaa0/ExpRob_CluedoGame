#!/usr/bin/env python

from armor_api.armor_client import ArmorClient
import rospy

if __name__ == '__main__':
    print("HELLO")
    path = "/root/Desktop/"
    client = ArmorClient("client", "reference")
    client.utils.load_ref_from_file(path + "cluedo_ontology.owl", "http://www.emarolab.it/cluedo-ontology",
                                True, "PELLET", True, False)  # initializing with buffered manipulation and reasoning
    client.utils.mount_on_ref()
    client.utils.set_log_to_terminal(True)
    
    client.manipulation.add_ind_to_class("Karl", "PERSON")
    client.manipulation.add_ind_to_class("Jim", "PERSON")
    
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()
    
    objects = client.query.ind_b2_class("PERSON")
    print(objects)
    
    # SAVE AND EXIT
    client.utils.save_ref_with_inferences(path + "cluedo_tested.owl")
