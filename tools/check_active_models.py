#!/usr/bin/env python3
"""
Check Active Models Script
Shows which vision and policy models are currently active on comma3x

Usage:
    python tools/check_active_models.py
"""

import os
from pathlib import Path
from openpilot.common.params import Params
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.system.hardware.hw import Paths

def check_active_models():
    """Check which models are currently active"""
    print("="*70)
    print("Active Models Check - Comma3x")
    print("="*70)
    
    params = Params()
    
    # Check active bundle
    print("\n1. Active Model Bundle:")
    bundle = get_active_bundle(params)
    
    if bundle:
        print(f"   Bundle Internal Name: {bundle.internalName}")
        print(f"   Bundle Display Name: {bundle.displayName}")
        print(f"   Runner Type: {bundle.runner}")
        print(f"   Is 20Hz: {bundle.is20hz}")
        print(f"   Generation: {bundle.generation}")
        print(f"   Models in bundle: {len(bundle.models)}")
        
        print("\n2. Models in Active Bundle:")
        for i, model in enumerate(bundle.models, 1):
            print(f"\n   Model {i}:")
            print(f"     Type: {model.type}")
            print(f"     Artifact: {model.artifact.fileName}")
            print(f"     Metadata: {model.metadata.fileName if model.metadata else 'None'}")
            
            # Check if files exist
            model_path = Path(Paths.model_root()) / model.artifact.fileName
            metadata_path = Path(Paths.model_root()) / model.metadata.fileName if model.metadata else None
            
            print(f"     Model file exists: {model_path.exists()} ({model_path})")
            if metadata_path:
                print(f"     Metadata file exists: {metadata_path.exists()} ({metadata_path})")
            
            if model_path.exists():
                size = model_path.stat().st_size / (1024 * 1024)  # MB
                print(f"     Model size: {size:.2f} MB")
    else:
        print("   No active bundle found - using default models")
    
    # Check default model paths
    print("\n3. Default Model Files:")
    default_models_dir = Path("/data/openpilot/selfdrive/modeld/models")
    
    vision_pkl = default_models_dir / "driving_vision_tinygrad.pkl"
    policy_pkl = default_models_dir / "driving_policy_tinygrad.pkl"
    vision_meta = default_models_dir / "driving_vision_metadata.pkl"
    policy_meta = default_models_dir / "driving_policy_metadata.pkl"
    
    print(f"   Vision model: {vision_pkl.exists()} ({vision_pkl})")
    if vision_pkl.exists():
        size = vision_pkl.stat().st_size / (1024 * 1024)
        print(f"     Size: {size:.2f} MB")
    
    print(f"   Policy model: {policy_pkl.exists()} ({policy_pkl})")
    if policy_pkl.exists():
        size = policy_pkl.stat().st_size / (1024 * 1024)
        print(f"     Size: {size:.2f} MB")
    
    print(f"   Vision metadata: {vision_meta.exists()} ({vision_meta})")
    print(f"   Policy metadata: {policy_meta.exists()} ({policy_meta})")
    
    # Check custom models directory
    print("\n4. Custom Models Directory:")
    custom_models_dir = Path(Paths.model_root())
    print(f"   Path: {custom_models_dir}")
    
    if custom_models_dir.exists():
        vision_models = list(custom_models_dir.glob("driving_vision_*_tinygrad.pkl"))
        policy_models = list(custom_models_dir.glob("driving_policy_*_tinygrad.pkl"))
        
        print(f"   Vision models found: {len(vision_models)}")
        for vm in sorted(vision_models)[:5]:  # Show first 5
            size = vm.stat().st_size / (1024 * 1024)
            print(f"     - {vm.name} ({size:.2f} MB)")
        if len(vision_models) > 5:
            print(f"     ... and {len(vision_models) - 5} more")
        
        print(f"   Policy models found: {len(policy_models)}")
        for pm in sorted(policy_models)[:5]:  # Show first 5
            size = pm.stat().st_size / (1024 * 1024)
            print(f"     - {pm.name} ({size:.2f} MB)")
        if len(policy_models) > 5:
            print(f"     ... and {len(policy_models) - 5} more")
    else:
        print(f"   Directory does not exist")
    
    # Check which runner is being used
    print("\n5. Model Runner Type:")
    from openpilot.sunnypilot.models.helpers import get_active_model_runner
    from openpilot.sunnypilot.models.runners.helpers import get_model_runner
    
    runner_type = get_active_model_runner(params)
    print(f"   Active runner type: {runner_type}")
    
    try:
        runner = get_model_runner()
        print(f"   Runner class: {runner.__class__.__name__}")
        
        if hasattr(runner, 'models'):
            print(f"   Loaded models: {list(runner.models.keys())}")
    except Exception as e:
        print(f"   Could not instantiate runner: {e}")
    
    print("\n" + "="*70)
    print("Summary:")
    if bundle:
        print(f"  Using custom bundle: {bundle.displayName} ({bundle.internalName})")
        model_types = []
        for m in bundle.models:
            type_name = "vision" if m.type.raw == 2 else "policy" if m.type.raw == 3 else "supercombo" if m.type.raw == 0 else f"type_{m.type.raw}"
            model_types.append(f"{type_name}: {m.artifact.fileName}")
        print(f"  Models:")
        for mt in model_types:
            print(f"    - {mt}")
    else:
        print(f"  Using default models:")
        print(f"    - Vision: driving_vision_tinygrad.pkl")
        print(f"    - Policy: driving_policy_tinygrad.pkl")
    print("="*70)


if __name__ == "__main__":
    check_active_models()

