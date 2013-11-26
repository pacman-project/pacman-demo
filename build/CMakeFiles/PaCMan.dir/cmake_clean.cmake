FILE(REMOVE_RECURSE
  "/home/pacman/CODE/lib/libPaCMan.pdb"
  "/home/pacman/CODE/lib/libPaCMan.a"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/PaCMan.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
