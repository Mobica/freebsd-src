#include <sys/param.h>
#include <sys/fcntl.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/namei.h>
#include <sys/proc.h>
#include <sys/sbuf.h>
#include <sys/vnode.h>

static int catfile(const char *filename)
{
  struct sbuf *sb;
  static char buf[128];
  struct nameidata nd;
  off_t ofs;
  ssize_t resid;
  int error, flags, len;

  NDINIT(&nd, LOOKUP, FOLLOW, UIO_SYSSPACE, filename);
  flags = FREAD;
  error = vn_open(&nd, &flags, 0, NULL);
  if (error)
    return (error);

  NDFREE_PNBUF(&nd);

  ofs = 0;
  len = sizeof(buf) - 1;
  // sb = sbuf_new_auto();
  sb = sbuf_new(NULL, NULL, len+1, SBUF_FIXEDLEN);
  if (!sb) {
    uprintf("LUKR: failed to allocate a storage buffer\n");
    return 1;
  }

  while (1) {
    error = vn_rdwr(UIO_READ, nd.ni_vp, buf, len, ofs,
                    UIO_SYSSPACE, IO_NODELOCKED, curthread->td_ucred,
                    NOCRED, &resid, curthread);
    if (error)
      break;
    if (resid == len)
      break;
    buf[len - resid] = 0;
    // uprintf("LUKR: %s\n", buf);
    sbuf_printf(sb, "%s", buf);
    

    ofs += len - resid;
  }

  sbuf_finish(sb);
  VOP_UNLOCK(nd.ni_vp);
  vn_close(nd.ni_vp, FREAD, curthread->td_ucred, curthread);
  uprintf("%s", sbuf_data(sb));
  // uprintf("%s\n", buf);
  return 0;
}

static int lukr_event_handler(struct module *module, int event, void *arg)
{
  int result = 0;

  switch (event)
  {
    case MOD_LOAD:
    {
#if defined(INVARIANTS) || defined(INVARIANT_SUPPORT)
      uprintf("LUKR: \"INVARIANTS\" Enabled !\n");
#else
      uprintf("LUKR: \"INVARIANTS\" Disabled !\n");
#endif
      uprintf("LUKR: module loading.\n");
      if (catfile("/etc/motd") != 0)
        uprintf("LUKR: Error reading MOTD.\n");
      break;
    }
    case MOD_UNLOAD:
    {
      uprintf("LUKR: module unloading.\n");
      break;
    }
    default:
      result = EOPNOTSUPP;
  }

  return result;
}

static moduledata_t  lukr_module_data = {
  "lukr_kmod",
  lukr_event_handler,
  NULL
};

DECLARE_MODULE(lukr_kmod, lukr_module_data, SI_SUB_DRIVERS, SI_ORDER_MIDDLE);