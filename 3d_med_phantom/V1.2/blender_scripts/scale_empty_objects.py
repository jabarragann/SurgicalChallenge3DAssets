

def scale_empty_objects(scale_factor):
    for obj in bpy.data.objects:
        if obj.type == 'EMPTY':
            current_size = obj.empty_display_size 
            obj.empty_display_size = current_size * scale_factor

def scale_and_move_selected_objects(scale_factor):
    for obj in bpy.context.selected_objects:
        obj.dimensions = obj.dimensions * scale_factor
        obj.location = obj.location * scale_factor
        if obj.type == 'EMPTY':
            obj.empty_display_size = obj.empty_display_size * scale_factor

