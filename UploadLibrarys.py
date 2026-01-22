"""
Upload library files to LEGO Spike Prime Hub using mpremote.

This script uploads DriveBase.py and Logger.py to the /flash/lib/ directory
on the LEGO hub, making them available for import in programs.
Uses a virtual environment for isolated mpremote installation.
"""

import subprocess
import sys
import os
import time
import glob
import argparse


# Virtual environment path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VENV_DIR = os.path.join(SCRIPT_DIR, '.venv')
VENV_PYTHON = os.path.join(VENV_DIR, 'Scripts' if os.name == 'nt' else 'bin', 'python')
VENV_MPREMOTE = os.path.join(VENV_DIR, 'Scripts' if os.name == 'nt' else 'bin', 'mpremote')


def ensure_venv() -> bool:
    """
    Ensure virtual environment exists and has mpremote installed.
    
    Returns:
        bool: True if venv is ready, False otherwise
    """
    # Create venv if it doesn't exist
    if not os.path.exists(VENV_DIR):
        print("Creating virtual environment...")
        try:
            result = subprocess.run(
                [sys.executable, '-m', 'venv', VENV_DIR],
                capture_output=True,
                text=True
            )
            if result.returncode != 0:
                print(f"✗ Failed to create virtual environment: {result.stderr}")
                return False
            print("✓ Virtual environment created")
        except Exception as e:
            print(f"✗ Error creating virtual environment: {e}")
            return False
    else:
        print("✓ Virtual environment exists")
    
    # Check if mpremote is installed in venv
    if os.path.exists(VENV_MPREMOTE):
        print("✓ mpremote is installed in virtual environment")
        return True
    
    print("Installing mpremote in virtual environment...")
    try:
        install_result = subprocess.run(
            [VENV_PYTHON, '-m', 'pip', 'install', 'mpremote'],
            capture_output=True,
            text=True
        )
        
        if install_result.returncode == 0:
            print("✓ mpremote installed successfully in virtual environment!")
            return True
        else:
            print(f"✗ Installation failed: {install_result.stderr}")
            return False
    except Exception as e:
        print(f"✗ Error during installation: {e}")
        return False


def find_spike_port() -> str | None:
    """Find the COM port of a LEGO Spike Prime hub."""
    try:
        # Pyserial is available in venv (installed by mpremote)
        cmd = (
            "import serial.tools.list_ports; "
            "ports = list(serial.tools.list_ports.comports()); "
            "spike = next((p.device for p in ports if '0694:' in p.hwid or 'LEGO' in p.description or 'Spike' in p.description), None); "
            "print(spike if spike else '')"
        )
        result = subprocess.run(
            [VENV_PYTHON, '-c', cmd],
            capture_output=True,
            text=True
        )
        return result.stdout.strip() or None
    except Exception:
        return None


def force_disconnect() -> None:
    """
    Attempt to disconnect any active mpremote connections.
    This helps when the hub is already connected (e.g., via VSCode extension).
    """
    # 1. Try polite mpremote disconnect first
    try:
        subprocess.run(
            [VENV_MPREMOTE, 'disconnect'],
            capture_output=True,
            text=True,
            timeout=2
        )
        time.sleep(0.5)
    except (subprocess.TimeoutExpired, Exception):
        pass

    # 2. Linux: aggressively free the serial port
    if sys.platform.startswith('linux'):
        ports = glob.glob('/dev/ttyACM*')
        if ports:
            port = ports[0] # Assume first Spike Prime hub
            try:
                check = subprocess.run(['fuser', port], capture_output=True, text=True)
                if check.stdout.strip():
                    print(f"  Found process locking {port}. Forcing disconnect...")
                    subprocess.run(['fuser', '-k', port], capture_output=True, text=True)
                    time.sleep(1.5)
                    print("  ✓ Port freed")
            except Exception as e:
                print(f"  Warning: Could not free serial port: {e}")
    
    # 3. Windows: Detect and Warn
    elif sys.platform == 'win32':
        # On Windows, we cannot easily force close handles from other processes
        # without admin rights. We can only advise the user.
        pass


def test_connection(retry: bool = True) -> bool:
    """
    Test connection to the LEGO hub.
    
    Args:
        retry: If True, attempt to disconnect and retry once on failure
    
    Returns:
        bool: True if connected, False otherwise
    """
    result = subprocess.run(
        [VENV_MPREMOTE, 'exec', "print('Connected')"],
        capture_output=True,
        text=True
    )
    
    # If failed and retry is enabled, try to force disconnect and reconnect
    if result.returncode != 0:
        if retry:
            print("  Initial connection failed, attempting to disconnect existing connections...")
            force_disconnect()
            return test_connection(retry=False)
        else:
            # Final failure handling
            if sys.platform == 'win32':
                port = find_spike_port()
                if port:
                    print(f"\n  ⚠  WARNING: Port {port} seems busy!")
                    print("     On Windows, you must manually disconnect any app")
                    print("     (like the VS Code Spike Extension) using this port.")
    
    return result.returncode == 0


def soft_reset() -> None:
    """
    Perform a reset on the hub to exit REPL mode and return to normal operation.
    This prevents the hub from staying in an interactive state after mpremote commands.
    """
    try:
        subprocess.run(
            [VENV_MPREMOTE, 'reset'],
            capture_output=True,
            text=True,
            timeout=3
        )
        # Give the hub a moment to reset
        time.sleep(1.5)
    except subprocess.TimeoutExpired:
        # Timeout is expected as the connection drops during reset
        pass
    except Exception:
        # Ignore errors during reset
        pass


def upload_libraries() -> bool:
    """
    Upload DriveBase.py and Logger.py to /flash/lib/ on the LEGO hub.
    
    Returns:
        bool: True if successful, False otherwise
    """
    drivebase_path = os.path.join(SCRIPT_DIR, 'DriveBase.py')
    logger_path = os.path.join(SCRIPT_DIR, 'Logger.py')
    
    # Verify files exist
    if not os.path.exists(drivebase_path):
        print(f"✗ Error: DriveBase.py not found at {drivebase_path}")
        return False
    
    if not os.path.exists(logger_path):
        print(f"✗ Error: Logger.py not found at {logger_path}")
        return False
    
    print("Uploading libraries to LEGO hub...")
    
    # Create /flash/lib directory and upload files
    commands = [
        VENV_MPREMOTE,
        'exec', "import os\ntry:\n    os.mkdir('/flash/lib')\nexcept OSError:\n    pass",
        '+',
        'cp', drivebase_path, ':/flash/lib/DriveBase.py',
        '+',
        'cp', logger_path, ':/flash/lib/Logger.py'
    ]
    
    result = subprocess.run(commands, capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✓ DriveBase.py uploaded successfully")
        print("✓ Logger.py uploaded successfully")
        return True
    else:
        print(f"✗ Upload failed: {result.stderr}")
        return False


def delete_programs() -> None:
    """Delete all programs in slots 00-19."""
    print("Deleting programs in slots 00-19...")
    
    # Python script to run on the hub to clear programs
    clear_script = (
        "import os\n"
        "for i in range(20):\n"
        "    d = '/flash/program/{:02d}'.format(i)\n"
        "    try:\n"
        "        os.remove(d + '/program.py')\n"
        "    except OSError:\n"
        "        pass\n"
        "    try:\n"
        "        os.rmdir(d)\n"
        "    except OSError:\n"
        "        pass\n"
    )
    
    subprocess.run(
        [VENV_MPREMOTE, 'exec', clear_script],
        capture_output=True,
        text=True
    )
    print("✓ Program slots cleared")


def upload_controller(slots: int) -> bool:
    """
    Upload Controller.py to the specified number of program slots.
    
    Args:
        slots: Number of slots to upload to (starting from 00)
    """
    controller_path = os.path.join(SCRIPT_DIR, 'Controller.py')
    if not os.path.exists(controller_path):
        print(f"✗ Error: Controller.py not found at {controller_path}")
        return False

    print(f"Uploading Controller.py to {slots} slot(s)...")
    
    success = True
    for i in range(slots):
        slot_dir = f"/flash/program/{i:02d}"
        remote_path = f"{slot_dir}/program.py"
        
        # Script to ensure directories exist
        # We chain the creation of base dir and slot dir
        mk_dirs_script = (
            "import os\n"
            "try:\n"
            "    os.mkdir('/flash/program')\n"
            "except OSError:\n"
            "    pass\n"
            "try:\n"
            f"    os.mkdir('{slot_dir}')\n"
            "except OSError:\n"
            "    pass"
        )
        
        # Chain commands: create dirs -> copy file (in one session)
        print(f"  Uploading to slot {i:02d}...", end='', flush=True)
        
        commands = [
            VENV_MPREMOTE,
            'exec', mk_dirs_script,
            '+',
            'cp', controller_path, ':' + remote_path
        ]
        
        result = subprocess.run(
            commands,
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            print(" ✓")
        else:
            print(f" ✗ Failed: {result.stderr.strip()}")
            success = False
            
    return success


def main():
    """Main execution function."""
    parser = argparse.ArgumentParser(description='LEGO Spike Prime Upload Tool')
    parser.add_argument('--slots', type=int, default=1, help='Number of program slots to upload Controller.py to (default: 1)')
    parser.add_argument('--reload', action='store_true', help='Re-upload library files (DriveBase.py, Logger.py)')
    parser.add_argument('--reset', action='store_true', help='Delete all programs in slots 00-19')
    parser.add_argument('--no-upload', action='store_true', help='Skip uploading Controller.py')
    args = parser.parse_args()

    print("=" * 50)
    print("LEGO Spike Prime - Upload Tool")
    print("=" * 50)
    print()
    
    # Ensure virtual environment is set up with mpremote
    if not ensure_venv():
        sys.exit(1)
    
    print()
    print("-" * 50)
    print()
    
    # Test connection
    print("Testing connection to LEGO hub...")
    if not test_connection():
        print("✗ Connection failed. Please ensure:")
        print("  1. The LEGO hub is connected via USB")
        print("  2. No other program is using the hub")
        sys.exit(1)
    
    print("✓ Connected to LEGO hub")
    print()
    print("-" * 50)
    print()
    
    # 1. Reset if requested
    if args.reset:
        delete_programs()
        print()

    # 2. Reload libraries if requested (was the default main action before)
    if args.reload:
        if not upload_libraries():
            print("✗ Library upload failed")
            sys.exit(1)
        print()

    # 3. Upload Controller to slots (Default action, unless reset or no-upload used)
    # User requested that reset should NOT result in an upload afterwards.
    should_upload = not args.no_upload and not args.reset

    if should_upload:
        if upload_controller(args.slots):
            print()
            print("=" * 50)
            print("Upload completed successfully!")
            print("=" * 50)
    else:
        print("Skipping Controller upload (--no-upload or --reset used)")
        
    # Soft reset to exit REPL mode (Always do this at the end)
    print("Resetting hub...")
    soft_reset()
    sys.exit(0)
    
    if upload_controller(args.slots):
        print()
        print("=" * 50)
        print("Upload completed successfully!")
        print("=" * 50)
        
        # Soft reset to exit REPL mode
        print("Resetting hub...")
        soft_reset()
        sys.exit(0)
    else:
        soft_reset()
        sys.exit(1)


if __name__ == "__main__":
    main()
