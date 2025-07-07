# Urbaxio Resources

This directory contains application resources such as icons, images, and other assets.

## Required Files

### Application Icon
- **`urbaxio_logo.ico`** - Windows icon file (ICO format) for the application executable
  - Recommended size: 256x256 pixels or multi-size ICO
  - This will appear in Windows Explorer, taskbar, and window title bar
  
### Source Images  
- **`urbaxio_logo.png`** - Original PNG logo image (can be any size)
  - Use this as source for creating the ICO file
  - Can also be used for splash screens or in-app branding

## Converting PNG to ICO

To convert your PNG logo to ICO format:

1. **Online converters**: Use services like convertio.co, icoconvert.com
2. **GIMP**: Export as .ico with multiple sizes
3. **ImageMagick**: `convert urbaxio_logo.png -resize 256x256 urbaxio_logo.ico`
4. **Windows**: Use online tools or specialized ICO editors

## File Structure
```
resources/
├── README.md           # This file
├── urbaxio.rc         # Windows resource definition
├── urbaxio_logo.ico   # Application icon (REQUIRED)
└── urbaxio_logo.png   # Source logo image
```

## Usage

The logo will be automatically embedded into `Urbaxio.exe` during build through the resource file `urbaxio.rc`. 