The following instructions explains how to use the download section (files) as an APT/deb Debian repository. The main problem is to avoid "./" in download URL, as Google Code won't serve such URI (404 not found).

So, in order to create or update the repository (delete, add files), all files must first be locally downloaded to a directory named "repo" (it'll be sed...).  Then, within this directory:

  * add new deb and source files packages. In order to know which source files to add for a given packages, see its `dsc` files. Usually, a `dsc`, `tar.gz` and `diff.gz` should be copied
  * if a package needs to be removed, delete it (also delete or deprecate it on Google Code site)
  * go to parent directory
  * `dpkg-scanpackages __repo__ | sed "s#__repo__/##" | gzip -9 - > Packages.gz`
  * `dpkg-scansources __repo__ | sed "s#__repo__##" |gzip -9 - > Sources.gz`

(careful, with dpkg-scansources, sed "repo", not "repo/" (slash)).


Upload new deb and source packages, as well as Packages.gz and Sources.gz. Don't bother about repo directory, all files can safely be "flatten" into GC download section.