// demo
#include <pcap.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

pcap_t *gHandle;

// close pcap handle to make pcap_loop() has error and break out 
void signalHandler(int sig)
{
    pcap_close(gHandle);
    printf("Close pcap handle\n");
}

// save packets callback
void saveCallback(u_char *arg, const struct pcap_pkthdr *pkthdr, const u_char *packet)
{
    pcap_dump(arg, pkthdr, packet);
}

int main(int argc, char **argv)
{
    // Set CTRL+C signal action
    signal(SIGINT, signalHandler);

    char err_buf[PCAP_ERRBUF_SIZE];
    pcap_if_t *all_dev, *dev;
    char *dev_select;

    // 0.Important: packets filter rule, modify this according to your application 
    // char filter_exp[] = ""; // capture all
    char filter_exp[] = "src host 192.168.0.2 || src host 192.168.0.55"; // only capture packets from lidar(192.168.0.2) or local(here is 192.168.0.55)

    // 1.Get devices list and select device to capture
    if (pcap_findalldevs(&all_dev, err_buf) == -1)
    {
        printf("Find all devices error: %s\n", err_buf);
        return 0;
    }
    int idx = 0, dev_id;
    for (dev = all_dev; dev; dev = dev->next)
    {
        printf("[%d] %s\n", ++idx, dev->name);
        if (dev->description)
        {
            printf("  %s\n", dev->description);
        }
    }
    printf("Please input device ID to capture: ");
    scanf("%d", &dev_id);
    if (dev_id < 1 || dev_id > idx)
    {
        printf("Device is unavailable\n");
        return 0;
    }
    for (dev = all_dev, idx = 1; idx <= dev_id; dev = dev->next, idx++)
        dev_select = dev->name;

    // 2.open device
    gHandle = pcap_open_live(dev_select, 65535, 1, 0, err_buf);
    if (!gHandle)
    {
        printf("Error: %s\n", err_buf);
        return 0;
    }

    // 3.set packets filter
    struct bpf_program filter;
    if (pcap_compile(gHandle, &filter, filter_exp, 1, 0) == -1)
    {
        printf("Error: cannot parse filter %s: %s\n", filter_exp, pcap_geterr(gHandle));
        return 0;
    }
    if (pcap_setfilter(gHandle, &filter) == -1)
    {
        printf("Error: cannot install filter %s: %s\n", filter_exp, pcap_geterr(gHandle));
        return 0;
    }

    // 4.open pcap output file and start capture loop
    pcap_dumper_t *out_pcap;
    out_pcap = pcap_dump_open(gHandle, "pack.pcap");
    if (!out_pcap)
    {
        printf("Error: cannot open pcap file\n");
        return 0;
    }
    printf("Capturing ...\n");
    pcap_loop(gHandle, -1, saveCallback, (u_char*)out_pcap); // second param -1 means loop will run untill error occurred

    // 5.flush buff and close file
    pcap_dump_flush(out_pcap);
    pcap_dump_close(out_pcap);

    printf("Finished\n");

    return 1;
}