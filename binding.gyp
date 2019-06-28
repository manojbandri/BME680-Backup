{
  "targets": [
    {
      "target_name": "<(module_name)",
      "sources": [
        "src/bme680_wrap.cc",
        "vendor/bsec_integration.c",
        "vendor/bme680.c",
        "vendor/bsec_serialized_configurations_iaq.c",
        "src/addon.cc",
      ],
      "libraries": [ "/home/nvidia/Desktop/bme680/vendor/libalgobsec.a" ],
      "include_dirs": [
        "<!(node -e \"require('nan')\")",
        "src",
        "vendor",
      ],
      'cflags_cc!': [ "-fno-rtti",  "-fno-exceptions"],
    },
    {
      "target_name": "action_after_build",
      "type": "none",
      "dependencies": [ "<(module_name)" ],
      "copies": [
        {
          "files": [ "<(PRODUCT_DIR)/<(module_name).node" ],
          "destination": "<(module_path)"
        }
      ]
    }
  ]
}
