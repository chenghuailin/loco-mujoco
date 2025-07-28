import os

# Test different backends
backends = ['osmesa', 'egl']

for backend in backends:
    print(f"\n{'='*50}")
    print(f"Testing {backend.upper()} backend...")
    print(f"{'='*50}")
    
    # Set the backend BEFORE importing mujoco
    os.environ['MUJOCO_GL'] = backend
    
    try:
        # Import mujoco fresh for each test
        import mujoco
        
        # Create a simple model
        model = mujoco.MjModel.from_xml_string("""
        <mujoco>
            <worldbody>
                <body>
                    <geom size='0.1'/>
                </body>
            </worldbody>
        </mujoco>
        """)
        
        data = mujoco.MjData(model)
        
        # Try to create a renderer
        renderer = mujoco.Renderer(model, height=100, width=100)
        
        print(f"✓ SUCCESS: {backend} backend works!")
        print(f"  Renderer type: {type(renderer).__name__}")
        
        # Try to render a frame
        renderer.update_scene(data)
        pixels = renderer.render()
        print(f"  Rendered frame shape: {pixels.shape}")
        
        # Clean up
        renderer.close()
        
        # Remove mujoco from cache to test next backend cleanly
        import sys
        if 'mujoco' in sys.modules:
            del sys.modules['mujoco']
        
    except Exception as e:
        print(f"✗ FAILED: {backend} backend failed")
        print(f"  Error: {e}")
        
        # Remove mujoco from cache to test next backend cleanly
        import sys
        if 'mujoco' in sys.modules:
            del sys.modules['mujoco']

print(f"\n{'='*50}")
print("Backend testing completed!")
print("Use the working backend in your MUJOCO_GL environment variable.")