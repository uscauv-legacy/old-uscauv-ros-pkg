uint8 POSITION_FLAG =    0 # moves seabee to the specified absolute position ( 0 ) or relative position ( 1 )
uint8 ORIENTATION_FLAG = 1 # makes seabee face the specified orientation ( 0 ) or the last position ( 1 )
						   # second values specifies absolute orientation ( 0 ) or relativel orientation ( 1 )
uint8 AREA_FLAG =        2 # limits movements to the specified area; centered at the last position ( 0 )
                           #                                         centered at the current position ( 1 )
uint8 SEARCH_FLAG =      3 # search (will describe later)

BehaviorFlag[] flags

# example: face given relative position
# flag: operation = 1         # 'set' operation
#       behavior = 0          # 'position' behavior
#       vector = { x, y, z }  # set the position
#       values = { 1 }        # use relative position
#
# flag: operation = 0         # 'execute' operation
#       behavior = 1          # 'orientation' behavior
#       vector = { }          # we're using the position calculated earlier so we can leave this blank
#       values = { 1 }        # calculate angle to the last position and face it
#
# example: move to given absolute position 
# flag: operation = 0         # 'execute' operation
#       behavior = 0          # 'position' behavior
#       vector = { x, y, z }  # the position to move to
#       values = { 0 }        # use absolute position
#
# example: face the given orientation; move to some absolute position; stay within some area relative to the current position
# flag: operation = 0         # 'execute' operation
#       behavior = 2          # set current operation area
#       vector = { x, y, z }  # values for the area
#       values = { 2 }        # relative to current position
#
# flag: operation = 0         # 'execute' operation
#       behavior = 1          # 'orientation' behavior
#       vector = { x, y, z }  # values
#       values = { 0 }        # face the specified angle
#
# flag: operation = 0         # 'execute' operation
#       behavior = 0          # 'position' behavior
#       vector = { x, y, z }  # position to move to
#       values = { 1 }